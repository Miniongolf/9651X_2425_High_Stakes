#include "autonFuncts.hpp"

namespace auton {
void chassisGrabRing(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
    lemlib::Pose newPose(
        x + intakeOffset * std::cos(lemlib::degToRad(90-theta)),
        y + intakeOffset * std::sin(lemlib::degToRad(90-theta)),
        theta
        );
    conveyor.forwards();
    activeChassis->moveToPose(newPose.x, newPose.y, theta, timeout, params, async);
}

void chassisGrabMogo(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
    lemlib::Pose newPose(
        x + mogoOffset * std::cos(lemlib::degToRad(90-theta)),
        y + mogoOffset * std::sin(lemlib::degToRad(90-theta)),
        theta
    );

    mogoMech.extend();
    lemlib::MoveToPoseParams newParams = params;
    newParams.forwards = false;
    activeChassis->moveToPose(newPose.x, newPose.y, theta, timeout, newParams, async);
    mogoMech.retract();
}


void leftWP() {
    activeChassis->setPose(-63, 16, -90);
    chassisGrabMogo(-24, 24, 45, 1000);
    activeChassis->turnToHeading(45, 1000);
    chassisGrabRing(-3.5, 43, 0, 2000);
    chassisGrabRing(-3.5, 55, 0, 2000);
    chassisGrabRing(-24, 48, 180, 2000);
    chassis.moveToPose(-24, 5, 180, 2000, {.maxSpeed = 75});
}

void leftMax() {}

void rightWP() {
    activeChassis->setPose(0, 0, 0);
    activeChassis->moveToPose(24, 0, 0, 1000);
}

void rightRush() {}

void skills() {}
}  // namespace auton
