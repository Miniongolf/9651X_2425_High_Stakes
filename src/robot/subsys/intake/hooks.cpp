#include "robot/subsys/intake/hooks.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"

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

bool Hooks::isJammed() const {
    if (!jamDetects[jamDetects.size() - 1]) return false;
    int count = 0;
    for (const auto& jam : jamDetects) {
        if (jam) count++;
    };
    return count >= 4;
}

void Hooks::update() {
    jamDetects.push_back(m_motor->get_current_draw() >= jamThresh.first &&
                         std::fabs(m_motor->get_actual_velocity()) <= jamThresh.second);
    if (jamDetects.size() > 5) jamDetects.erase(jamDetects.begin());

    if (this->isJammed()) { std::printf("HOOKS JAMMED\n"); currState = states::UNJAM; }

    switch (currState) {
        case states::FORWARDS: setVoltage(127); break;
        case states::REVERSE: setVoltage(-127); break;
        case states::IDLE: setVoltage(0); break;
        case states::UNJAM:
            setVoltage(lemlib::sgn(prevVoltage) * -50);
            m_motor->move(currVoltage);
            pros::delay(250);
            setState(prevState);
            break;
    }

    m_motor->move(currVoltage);
    prevVoltage = currVoltage;
    prevState = (lastState == currState) ? prevState : lastState;
    lastState = currState;
};
