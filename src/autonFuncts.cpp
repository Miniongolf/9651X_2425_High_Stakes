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
    robot::chassisSetPose(-64*s, 12, 180*s);
    robot::chassisGrabMogo({-24*s, 24, 60*s}, 2000, {.lead=0.1, .maxSpeed=100});
    robot::chassisGrabRing({-28*s, 48, 0*s}, 2000, {.lead=0, .maxSpeed=100});
    activeChassis->moveToPose(-35*s, 35, 45*s, 2000, {.forwards=false});
    pros::delay(500);
    robot::chassisGrabRing({-8*s, 45, 85*s}, 2000, {.lead=0});
    pros::delay(1500);
    activeChassis->moveToPose(-24*s, 48, 90*s, 1000, {.forwards=false});
    conveyor.queueIndex(1);
    pros::delay(500);
    robot::chassisGrabRing({-4*s, 51, 90*s}, 2000);
    activeChassis->turnToPoint(0, 70, 1000);
    pros::delay(1000);

//    robot::chassisGrabRing({-4*s, 60, 100*s}, 2000, {.lead=1, .maxSpeed=70});

//    activeChassis->moveToPose(48*s, 36, 135*s, 1250, {.forwards = false, .maxSpeed = 127});
//    activeChassis->turnToPoint(-24*s, 24, 600, {.forwards = false});
//    robot::chassisGrabMogo({-24*s, 24, 60*s}, 1500, {.lead = 0, .maxSpeed = 127});
//    robot::chassisPrintPose();

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

void tunePID() {
    robot::chassisSetPose(0, 0, 0);
    activeChassis->moveToPose(0, 24, 0, 2000);
    robot::chassisPrintPose();
    activeChassis->turnToHeading(90, 2000);
    robot::chassisPrintPose();
}
}  // namespace auton
