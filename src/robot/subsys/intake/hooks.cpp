#include "robot/subsys/intake/hooks.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"

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
    double hookPose = (hookNum == std::clamp(hookNum, 0, (int)hooks.size() - 1)) ? getPosition(hookNum)
                                                                                 : getPosition(getNearestHook(target));
    return std::fabs(dist(target, getPosition(hookNum)) < tolerance);
}

bool Hooks::isJammed() const {
    if (!jamDetects[jamDetects.size() - 1]) return false;
    int count = 0;
    for (const auto& jam : jamDetects) {
        if (jam) count++;
    };
    return count >= 4;
}

void Hooks::update(bool hasPrerollRing) {
    /** Jam detection
     *  NOTE: the jam state is intermediary and will not be tracked by lastState or prevState
     *  However, it will be tracked by prevVoltage
     */
    jamDetects.push_back(m_motor->get_current_draw() >= jamThresh.first &&
                         std::fabs(m_motor->get_actual_velocity()) <= jamThresh.second);
    if (jamDetects.size() > 5) {
        jamDetects.erase(jamDetects.begin());
    } // Keep the vector size at 5 by popping the oldest element
    if (this->isJammed()) {
        std::printf("HOOKS JAMMED\n");
        // currVoltage at this stage is the most recently set voltage
        setVoltage(lemlib::sgn(currVoltage) * -100);
        m_motor->move(currVoltage);
        // clear the jam detections
        jamDetects = {false, false, false, false, false};
        pros::delay(250);
    }

    lemlib::AngularDirection currentDirection =
        currVoltage >= 0 ? AngularDirection::CW_CLOCKWISE : AngularDirection::CCW_COUNTERCLOCKWISE;

    switch (currState) {
        case states::FORWARDS:
            setVoltage(127);
            nextState();
            break;
        case states::REVERSE:
            setVoltage(-127);
            nextState();
            break;
        case states::IDLE:
            if (this->isAtPosition(idlePose)) setVoltage(0);
            else {
                // Move the nearest hook to idle position, continuing in the current direction
                setVoltage(pid.update(
                    dist(idlePose, getPosition(getNearestHook(idlePose, currentDirection)), currentDirection)));
            }
            nextState();
            break;
        case states::WAIT_FOR_RING:
            // Move into position first even if there is a ring already
            if (!this->isAtPosition(idlePose)) {
                // Move the nearest hook to idle position, moving forwards
                setVoltage(pid.update(
                    dist(idlePose, getPosition(getNearestHook(idlePose, lemlib::AngularDirection::CW_CLOCKWISE)),
                         lemlib::AngularDirection::CW_CLOCKWISE)));
            } else if (!hasPrerollRing) {
                setVoltage(0);
            } else {
                // Go if there is a ring and the hooks are in position
                nextState();
            }
            break;
        case states::MOVE:
            setState(states::IDLE);
            nextState();
            break;
    }

    m_motor->move(currVoltage);
    prevVoltage = currVoltage;
    prevState = (lastState == currState) ? prevState : lastState;
    lastState = currState;
};
