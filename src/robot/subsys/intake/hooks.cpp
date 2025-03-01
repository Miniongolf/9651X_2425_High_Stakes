#include "robot/subsys/intake/hooks.hpp"
#include "lemlib/chassis/chassis.hpp"

void Hooks::setState(states state, bool forceInstant, bool clearQueue) {
    if (forceInstant) {
        currState = state;
    } else {
        if (clearQueue) { stateQueue.clear(); }
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
    float sensorRotations = m_rotSensor->get_position() * (1.0 / 36000);
    if (m_rotSensor->get_position() == 2147483647) {
        // Sensor dc handling, use motor encoders (might drift)
        return sanitizePosition(m_motor->get_position() * 12 + hooks[hookNum] + poseOffset);
        std::printf("HOOKS ROTATION DC\n");
    } else {
        // Using rotation sensor
        return sanitizePosition(sensorRotations * 12 + hooks[hookNum] + poseOffset);
    }
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

int Hooks::getNearestHook(double target, lemlib::AngularDirection direction, double tolerance) const {
    double minDist = chainLength;
    int minHook = 0;
    for (int i = 0; i < hooks.size(); i++) {
        // Check for tolerance
        if (std::fabs(dist(target, getPosition(i), lemlib::AngularDirection::AUTO)) < tolerance) { return i; }
        // Search for nearest hook
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
    double hookPose = getPosition(hookNum);
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

Alliance Hooks::ringDetect() const {
    const double ringHue = m_optical->get_hue();
    const int ringProx = m_optical->get_proximity();
    // Error handling for invalid or dced optical
    // std::printf("hooks ringDetect: %f %d\n", ringHue, ringProx);
    if (ringProx == 2147483647) {
        return Alliance::NONE;
        std::printf("OPTICAL DC\n");
    }
    // Proximity check
    if (ringProx < proxRange) return Alliance::NONE;
    if (red.inRange(ringHue)) return Alliance::RED;
    else if (blue.inRange(ringHue)) return Alliance::BLUE;
    else return Alliance::NONE;
}

void Hooks::moveTowards(double target, int hookNum, lemlib::AngularDirection direction, double settleRange) {
    // If hookNum is -1 (or otherwise invalid), use the nearest hook
    if (hookNum < 0 || hookNum >= (int)hooks.size()) { hookNum = getNearestHook(target, direction, settleRange); }
    // Check if the hook is settling
    if (isAtPosition(target, hookNum, settleRange)) { direction = AngularDirection::AUTO; }
    // Move the hook to the target position
    const double error = dist(target, getPosition(hookNum), direction);
    const double voltage = pid.update(error);
    setVoltage(voltage);
};

void Hooks::update(bool hasPrerollRing, bool forcedIndex, bool isArmDown, bool isArmUp, bool isArmStuck) {
    /** Jam detection
     *  NOTE: the jam state is not a real state and will not be tracked by lastState or prevState
     *  However, its effects will be tracked by prevVoltage
     */
    jamDetects.push_back(m_motor->get_current_draw() >= jamThresh.first &&
                         std::fabs(m_motor->get_actual_velocity()) <= jamThresh.second && currVoltage != 0);
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

    lemlib::AngularDirection currentDirection = prevIsArmStuck     ? lemlib::AngularDirection::AUTO
                                                : currVoltage >= 0 ? AngularDirection::CW_CLOCKWISE
                                                                   : AngularDirection::CCW_COUNTERCLOCKWISE;

    // int maxVolt = 90;
    int maxVolt = (isArmUp) ? 127 : 100;
    int idlePose = (isArmDown) ? 0 : -5;

    if (forcedIndex) { setState(states::INDEX, true, true); }

    // std::printf("HOOKS DETECT: %d\n", ringDetect());

    switch (currState) {
        case states::FORWARDS:
            setVoltage(maxVolt);
            // Detect colour sorts
            if (colourSortEnabled && isOpposite(m_alliance, ringDetect())) {
                std::printf("HOOKS COLOUR SORT INITIATED\n");
                colourDetectHook = getNearestHook(colourSortPose, lemlib::AngularDirection::CW_CLOCKWISE);
                lemlib::Timer colourSortTimer(500);
                while (dist(colourSortPose, getPosition(colourDetectHook)) > 0 && !colourSortTimer.isDone()) {
                    pros::delay(10);
                }
                std::printf("EJECTING RING\n");
                m_motor->move(-10);
                pros::delay(200);
                m_motor->move(maxVolt);
            }
            break;
        case states::REVERSE:
            setVoltage(-maxVolt);
            isBusy = false;
            break;
        case states::IDLE:
            // Prioritize moving the arm up if it is stuck
            if (isArmStuck) {
                setVoltage(60);
            } else if (this->isAtPosition(idlePose)) {
                setVoltage(0);
            } else {
                moveTowards(idlePose, -1, currentDirection, 7);
            }
            isBusy = false;
            break;
        case states::INDEX:
            if (isArmUp) setState(states::IDLE, true, true);
            // reset ring wait flag when the state is first set
            if (prevState != states::INDEX) {
                sawPrerollRing = false;
                indexHook = getNearestHook(idlePose, AngularDirection::CW_CLOCKWISE);
            }

            hasPrerollRing = hasPrerollRing || forcedIndex;
            if (hasPrerollRing) sawPrerollRing = true;

            if (hasPrerollRing && !prevHasPrerollRing) { std::cout << "HOOKS INDEX DETECTED\n"; }
            // Move into position first even if there is a ring already
            if (dist(idlePose, getPosition(indexHook)) > 2) {
                moveTowards(idlePose, indexHook);
            } else if (sawPrerollRing) {
                if (this->isAtPosition(idlePose + 18, indexHook)) {
                    setState(states::IDLE, true, true);
                } else {
                    moveTowards(idlePose + 18, indexHook);
                }
            } else {
                setVoltage(0);
            }

            if (forcedIndex) {
                m_motor->move(80);
                pros::delay(200);
            }
            break;
    }
    m_motor->move(currVoltage);
    prevHasPrerollRing = hasPrerollRing;
    prevIsArmStuck = isArmStuck;
    prevVoltage = currVoltage;
    prevState = (lastState == currState) ? prevState : lastState;
    lastState = currState;
    // if (!isBusy) nextState();
};