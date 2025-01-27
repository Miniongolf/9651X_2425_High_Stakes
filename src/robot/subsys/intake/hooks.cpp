#include "robot/subsys/intake/hooks.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "robot/constants.hpp"
#include "robot/globals.hpp"

void Hooks::setState(states state, bool forceInstant, bool clearQueue) {
    if (forceInstant) {
        currState = state;
    } else {
        if (clearQueue) {
            stateQueue.clear();
        }
        stateQueue.push_back(state);
    };
}

void Hooks::nextState() {
    if (stateQueue.empty()) {
        return;
    } else {
        currState = stateQueue.front();
        stateQueue.pop_front();
    }
}

double Hooks::getPosition(int hookNum) const {
    hookNum = std::clamp(hookNum, 0, (int)hooks.size() - 1); // Sanitize hooknum input
    // Using motor position in revolutions on a 12t sprocket
    // Return sanitized position of chain links moved (revs * sprocket teeth)
    return sanitizePosition(m_motor->get_position() * 12 + hooks[hookNum]);
}

double Hooks::dist(double target, double position, lemlib::AngularDirection direction) const {
    // Based off LemLib angle error function logic
    // Sanitize inputs
    target = sanitizePosition(target);
    position = sanitizePosition(position);

    // Raw error
    const double rawError = target - position;

    // Direction handling
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + chainLength : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - chainLength : rawError; // subtract max if sign does not match
        default: // AUTO, choose the shortest path
            return std::remainder(rawError, chainLength);
    }
}

int Hooks::getNearestHook(double target, lemlib::AngularDirection direction) const {
    double minDist = chainLength;
    int minHook = 0;
    for (int i = 0; i < hooks.size(); i++) {
        if (std::fabs(dist(target, getPosition(i), direction)) < minDist) {
            minDist = std::fabs(dist(target, getPosition(i), direction));
            minHook = i;
        }
    }
    return minHook;
}

bool Hooks::isAtPosition(double target, int hookNum, double tolerance) const {
    // If hookNum is -1 (or otherwise invalid), use the nearest hook
    if (hookNum < 0 || hookNum >= (int)hooks.size()) { hookNum = getNearestHook(target); }   
    double hookPose =  getPosition(hookNum);
    return std::fabs(dist(target, getPosition(hookNum))) < tolerance;
}

bool Hooks::isJammed() const {
    if (!jamDetects[jamDetects.size() - 1]) return false;
    int count = 0;
    for (const auto& jam : jamDetects) {
        if (jam) count++;
    };
    return count >= 4;
}

void Hooks::moveTowards(double target, int hookNum, lemlib::AngularDirection direction, double settleRange) {
    // If hookNum is -1 (or otherwise invalid), use the nearest hook
    if (hookNum < 0 || hookNum >= (int)hooks.size()) { hookNum = getNearestHook(target); }
    // Check if the hook is settling
    if (isAtPosition(target, hookNum, settleRange)) { direction = AngularDirection::AUTO; }
    // Move the hook to the target position
    const double error = dist(target, getPosition(hookNum), direction);
    const double voltage = pid.update(error);
    setVoltage(voltage);
};

void Hooks::update(bool hasPrerollRing) {
    /** Jam detection
     *  NOTE: the jam state is not a real state and will not be tracked by lastState or prevState
     *  However, its effects will be tracked by prevVoltage
     */
    jamDetects.push_back(m_motor->get_current_draw() >= jamThresh.first &&
                         std::fabs(m_motor->get_actual_velocity()) <= jamThresh.second);
    if (jamDetects.size() > 5) {
        jamDetects.erase(jamDetects.begin());
    } // Keep the vector size at 5 by popping the oldest element
    if (this->isJammed()) {
        std::printf("HOOKS JAMMED\n");
        // currVoltage at this stage is the most recently set voltage
        setVoltage(lemlib::sgn(currVoltage) * -70);
        m_motor->move(currVoltage);
        // clear the jam detections
        jamDetects = {false, false, false, false, false};
        pros::delay(250);
    }

    lemlib::AngularDirection currentDirection =
        currVoltage >= 0 ? AngularDirection::CW_CLOCKWISE : AngularDirection::CCW_COUNTERCLOCKWISE;

    // Only colour sort if state is forwards
    if (currState != states::FORWARDS) { colourSorting = false; }

    switch (currState) {
        case states::FORWARDS:
            setVoltage(127);
            // Detect colour sorts
            if (colourSortEnabled && !colourSorting && isOpposite(ringDetect(), robotAlliance)) {
                colourSorting = true;
                colourDetectHook = getNearestHook(colourSortPose, lemlib::AngularDirection::CW_CLOCKWISE);
            }
            // Do the colour sort thingy if it's detected and the hook is in position
            if (colourSorting && dist(colourSortPose, getPosition(colourDetectHook)) < 0) {
                colourSorting = false;
                m_motor->move(-10);
                pros::delay(50);
                m_motor->move(127);
            }
            isBusy = colourSorting;
            break;
        case states::REVERSE:
            setVoltage(-127);
            isBusy = false;
            break;
        case states::IDLE:
            if (this->isAtPosition(idlePose)) setVoltage(0);
            else { moveTowards(idlePose, -1, currentDirection); }
            isBusy = false;
            break;
        case states::WAIT_FOR_RING:
            isBusy = true;
            // reset ring wait flag when the state is first set
            if (prevState != states::WAIT_FOR_RING) { ringWaitFlag = false; }
            // update flag with ring detection from function param
            if (hasPrerollRing) ringWaitFlag = true;

            // Move into position first even if there is a ring already
            if (!this->isAtPosition(idlePose, -1, 1)) {
                moveTowards(idlePose, -1, AngularDirection::CW_CLOCKWISE);
            } else if (!ringWaitFlag) {
                setVoltage(0);
            } else {
                // Hooks are in position and a ring was detected, continue
                isBusy = false;
            }
            break;
        case states::MOVE:
            /** NOTE: idrk if this is necessary at all, just immediately switches to idle for now */
            setState(states::IDLE);
            isBusy = false;
            break;
    }
    m_motor->move(currVoltage);
    prevVoltage = currVoltage;
    prevState = (lastState == currState) ? prevState : lastState;
    lastState = currState;
    if (!isBusy) nextState();
};
