#include "autonFuncts.hpp"

namespace auton {
void leftWP() {
    activeChassis->setPose(-61, 16, -90);
    robot::chassisGrabMogo(-24, 24, 45, 1000);
    activeChassis->turnToHeading(45, 1000);
    robot::chassisGrabRing(-3.5, 43, 0, 2000);
    robot::chassisGrabRing(-3.5, 55, 0, 2000);
    robot::chassisGrabRing(-24, 48, 180, 2000);
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
