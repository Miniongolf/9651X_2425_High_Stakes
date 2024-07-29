#include "autonFuncts.hpp"

namespace auton {
void scoreWP() {
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

void scoreMax() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-64*s, 12, 180*s);
    robot::chassisGrabMogo({-24*s, 24, 60*s}, 2000, {.lead=0.1, .maxSpeed=100});
    // Close ring
    robot::chassisGrabRing({-28*s, 48, 0*s}, 2000, {.lead=0, .maxSpeed=100});
    activeChassis->moveToPose(-35*s, 35, 45*s, 2000, {.forwards=false});
    pros::delay(500);
    // Mid ring 1
    robot::chassisGrabRing({-8*s, 45, 85*s}, 2000, {.lead=0});
    pros::delay(500);
    activeChassis->moveToPose(-24*s, 48, 90*s, 1000, {.forwards=false});
    pros::delay(500);
    // Mid ring 2
    robot::chassisGrabRing({-8*s, 51, 90*s}, 2000);
    pros::delay(1000);
    robot::setPTO(true);
    conveyor.stop();
    robot::chassisMove(50, 0, 200);
    // Bar touch
    activeChassis->moveToPose(-24*s, 24, 135*s, 5000, {.forwards=true, .lead=0, .maxSpeed=70});
    robot::chassisPrintPose();
    activeChassis->moveToPose(-15*s, 8.5, 135*s, 5000, {.forwards=true, .lead=0, .maxSpeed=70});
    pros::delay(700);
    arm.moveToAngle(20);
    robot::chassisPrintPose();
}

void rushWP() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-64*s, -12, 0);
    robot::chassisGrabMogo({-24*s, -24, -60*s}, 2000, {.lead=0.1, .maxSpeed=100});
    robot::chassisGrabRing({-28*s, -48, -0*s}, 2000, {.lead=0, .maxSpeed=100});
    activeChassis->turnToHeading(0, 1000);
    robot::setPTO(true);
    activeChassis->moveToPose(-12*s, -12, 45*s, 2000, {.maxSpeed=50});
    activeChassis->waitUntilDone();
    arm.moveToAngle(20);
    robot::chassisPrintPose();
    pros::delay(5000);
    arm.moveToAngle(arm.getAngle());
    pros::delay(2000);
}

void rushRush() {}

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
