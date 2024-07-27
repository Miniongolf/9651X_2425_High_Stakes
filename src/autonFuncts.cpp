#include "autonFuncts.hpp"

namespace auton {
void leftWP() {
    activeChassis->setPose(-56.5, 16.5, -90);
    robot::chassisGrabMogo({-24, 24, 60}, 3000);
    robot::chassisPrintPose();
    activeChassis->turnToHeading(-45, 1000);
    robot::chassisGrabRing({-24, 48, 0}, 3000);
    activeChassis->moveToPose(-24, 24, 0, 2000);
    robot::chassisGrabRing({-3.5, 43, 90}, 2000);
    robot::chassisGrabRing({-3.5, 55, 135}, 2000);
//    activeChassis->turnToHeading(45, 1000);
//    robot::chassisGrabRing(-24, 48, 180, 2000);
//    activeChassis->moveToPose(-24, 5, 180, 2000, {.maxSpeed = 75});
}

void leftMax() {}

void rightWP() {
}

void rightRush() {}

void skills() {
    robot::chassisSetPose(0, 0, 0);
//    activeChassis->moveToPose(0, 36, 0, 10000);
//    robot::chassisPrintPose();
//    pros::delay(1000);
//    activeChassis->turnToHeading(90, 10000);
    activeChassis->moveToPose(-24, 36, -90, 10000);

    robot::chassisPrintPose();
}
}  // namespace auton
