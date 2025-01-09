#include "robot/subsys/intake/hooks.hpp"
#include "lemlib/chassis/chassis.hpp"

void Hooks::initialize() {
    m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

bool Hooks::isJammed(int countThresh) const {
    countThresh = std::clamp(countThresh, 1, 5);
    int jamCount = 0;
    for (const auto& jammed : jamDetects) {
        if (jammed) jamCount++;
    }
    return (jamCount >= countThresh);
}

double Hooks::getPosition(int hookNum) const {
    hookNum = std::clamp(hookNum, 0, (int)hooks.size() - 1); // Sanitize hooknum input
    // Using motor position in revolutions on a 12t sprocket
    // Return sanitized position of chain links moved (revs * sprocket teeth)
    return sanitizePosition(m_motor->get_position() * 12 + hooks[hookNum]);
}

int Hooks::getNearestHook(double target, lemlib::AngularDirection direction) const {
    // Handle AUTO case
    if (direction == lemlib::AngularDirection::AUTO) {
        return std::min(getNearestHook(target, lemlib::AngularDirection::CW_CLOCKWISE),
                        getNearestHook(target, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE));
    }

    // Find distance from each hook to the target position
    std::vector<double> shiftedHookPositions = hooks;
    for (int i = 0; i < hooks.size(); i++) { shiftedHookPositions[i] = this->getPosition(i); }

    // Find the closest hook's distance
    double minDist = chainLength;
    int hookNum = 0;
    for (auto& hookPose : shiftedHookPositions) { minDist = std::min(minDist, dist(target, hookPose)); }
    return minDist;
}

double Hooks::dist(double target, double position, lemlib::AngularDirection direction) const {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizePosition(target);
    position = sanitizePosition(position);
    const double max = chainLength;
    const double rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // AUTO, choose the shortest path
            return std::remainder(rawError, max);
    }
}

double Hooks::moveToPosition(double position, int hookNum, lemlib::AngularDirection direction) {
    // p loop, exit range 0.5 links
    double error = dist(position, getNearestHook(position, direction), direction);
    return std::clamp(error * kP, -127.0, 127.0);
}

void Hooks::update(Alliance sortAlliance, Alliance detectedRing, bool isArmUp) {
    // Update jam detection queue
    jamDetects.push_back(m_motor->get_current_draw() >= jamThresh.first &&
                         m_motor->get_actual_velocity() <= jamThresh.second);
    if (jamDetects.size() > 5) jamDetects.erase(jamDetects.begin()); // Keep the 5 most recent values

    // Check for jam
    if (isJammed()) { m_state = states::UNJAM; } // unjam no matter what

    // Colour sort if an alliance colour is selected and the arm is down
    if (isOpposite(sortAlliance, detectRing()) && !isArmUp) { m_state = states::COLOUR_SORT; }

    double target = 0;

    // Handle state machine
    switch (m_state) {
        case states::FORWARDS: setVoltage(127); break;
        case states::REVERSE: setVoltage(-127); break;
        case states::IDLE: setVoltage(0); break;
        case states::UNJAM:
            if (jamTimer.isPaused()) {
                jamTimer.resume();
                isBusy = true;
                setVoltage(prevStateVoltage > 0 ? -50 : 50); // move in the opposite direction
            } else if (jamTimer.isDone()) {
                jamTimer.reset();
                jamTimer.pause();
                isBusy = false;
                m_state = (prevState == states::UNJAM) ? states::IDLE : prevState; // return to what it was doing before the jam
                // If the previous state is somehow unjam, set to idle to escape the loop
            }
            break;
        case states::WAIT_FOR_RING:
            isBusy = true;
            if (!isAtPosition(ringWaitPosition)) {
                setVoltage(moveToPosition(ringWaitPosition));
            } else if ((bool)detectRing()) {
                m_state = states::INDEX_UP;
                isBusy = false;
            }
            break;
        case states::INDEX_UP:
            isBusy = true;
            target = getPosition() + hooks[1] - hooks[0];
            // Move the next hook to the init hook's current position
            // clockwise direction
            setVoltage(moveToPosition(target, -1, lemlib::AngularDirection::CW_CLOCKWISE));
            m_state = nextState();
            isBusy = false;
            break;
        case states::INDEX_DOWN:
            isBusy = true;
            target = getPosition() + hooks.back() - hooks[0];
            // Move the previous hook to the init hook's current position
            setVoltage(moveToPosition(target, -1, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE));
            m_state = nextState();

            isBusy = false;
            break;
        case states::COLOUR_SORT:
            isBusy = true;
            // TODO: implement this
            setVoltage(moveToPosition(
                colourSortPosition,
                std::remainder(getNearestHook(colourSortPosition, lemlib::AngularDirection::CW_CLOCKWISE) - 1,
                               hooks.size() - 1),
                lemlib::AngularDirection::CW_CLOCKWISE));
            m_state = nextState();
            isBusy = false;
            break;
    }
    m_motor->move(currentMotorVoltage);
    lastState = m_state;
    prevState = (lastState == m_state) ? prevState : lastState;
    prevMotorVoltage = currentMotorVoltage;
    prevStateVoltage = (prevState == m_state) ? prevStateVoltage : prevMotorVoltage;
}
