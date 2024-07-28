#include "autonFuncts.hpp"

namespace auton {
void leftWP() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-56.5*s, 16.5, -90*s);
    robot::chassisGrabMogo({-24*s, 24, 60*s}, 2500);
    robot::chassisPrintPose();
    activeChassis->turnToHeading(-45*s, 1000);
    robot::chassisGrabRing({-24*s, 48, 0*s}, 3000);
    activeChassis->moveToPose(-24*s, 24, 0, 2000);
    robot::chassisGrabRing({-4*s, 43, 90*s}, 2000);
    robot::chassisGrabRing({-4*s, 55, 135*s}, 2000);
//    activeChassis->turnToHeading(45, 1000);
//    robot::chassisGrabRing(-24, 48, 180, 2000);
//    activeChassis->moveToPose(-24, 5, 180, 2000, {.maxSpeed = 75});
}

void leftMax() {
//    float s = isRedAlliance ? 1 : -1;
    float s = -1;
    conveyor.queueIndex(2);
    robot::chassisSetPose(-64*s, 12, 180*s);
    robot::chassisMove(-70, 0, 500);
    activeChassis->turnToHeading(45*s, 1500);
//    robot::chassisGrabRing({-24*s, 48, 95 * s}, 3000);
//    robot::chassisGrabRing({-4*s, 43, 90*s}, 2000);
//    activeChassis->turnToPoint(0, 70, 2000);
//    robot::chassisWallStake({0, 70, 45*s}, 4000);
//    robot::chassisMove(-70, 0, 500);

//    robot::chassisGrabRing({-4*s, 50, 90*s}, 2000);
//    activeChassis->moveToPose(-18*s, 38, 45*s, 1500);
//    robot::chassisGrabRing({-4*s, 55, 90*s}, 2000);
//    robot::setPTO(true);
//    robot::chassisWallStake({0, 69, 45*s}, 3000);
}

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
