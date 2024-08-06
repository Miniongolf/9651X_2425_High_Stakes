#include "autonFuncts.hpp"

namespace auton {
void scoreDisrupt() {
    float s = 1;
    robot::chassisSetPose(-56, 56, 90);
    intake.move(127);
    activeChassis->moveToPose(-20, 62, 90, 1000, {.minSpeed=127, .earlyExitRange=10});
    activeChassis->waitUntilDone();
    activeChassis->moveToPose(-4, 51, 135, 1000);
    pros::delay(500);
    robot::chassisGrabMogo({-14, 14, 75}, 1000);
    pros::delay(1000);
    robot::chassisGrabRing({-24, 48, 0}, 1000, {.minSpeed=70});
    pros::delay(500);
    activeChassis->turnToPoint(-70, 70, 1000);
    mogoMech.extend();
    robot::chassisMove(50, 0, 750);
};

void scoreWP() {
    float s = isRedAlliance ? 1 : -1;
    s = 1;
    robot::chassisSetPose(-55*s, 32, -90*s);
    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 50*s}, 900, {.lead=0.2});
    robot::chassisGrabRing({-24*s, 48, 0*s}, 2000);
    activeChassis->moveToPose(-54*s, -6, 180*s, 2000);
    activeChassis->waitUntil(36);
    mogoMech.extend();
    robot::chassisGrabMogo({-24*s, -24, -60*s}, 2000, {.lead=0.2});
    robot::chassisGrabRing({-24*s, -48, 0*s}, 2000, {.maxSpeed=100});
    activeChassis->moveToPose(-15*s, -15, 45*s, 2000, {.maxSpeed=50});
    robot::setPTO(true);
    activeChassis->waitUntilDone();
    arm.moveToAngle(15);
    pros::delay(5000);
}

void scoreMax() {
    conveyor.resetIndexQueue();
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-55*s, 41, -90*s);
    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 160*s}, 1000, {.lead=0.1, .maxSpeed=100});
    robot::chassisStop();
    pros::delay(600);
    // Ring 1
    robot::chassisGrabRing({-7*s, 43, 80*s}, 1750, {.maxSpeed=100});
//    // Ring 2
//    activeChassis->moveToPose(-18*s, 24, 0*s, 1500, {.forwards=false});
//    robot::chassisGrabRing({-24*s, 48, 0*s}, 1750, {.maxSpeed=100});
    pros::delay(1500);
    // Bar Touch
    activeChassis->moveToPose(-5*s, -15, 90*s, 2000, {.maxSpeed=70});
    activeChassis->waitUntil(24);
    mogoMech.extend();
    activeChassis->waitUntilDone();
//    robot::setPTO(true);
//    pros::delay(1000);
//    arm.moveToAngle(8);
//    activeChassis->moveToPose(-17, 17, 135, 2000, {.lead=0, .minSpeed=70});
//    activeChassis->waitUntil(10);
//    mogoMech.extend();
//    pros::delay(5000);
}

void scoreRush() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-55*s, 16, -90*s);
    robot::chassisGrabMogo({0, -48, -60*s}, 2000, {.minSpeed=100});
    robot::chassisPrintPose();
}

void rushWP() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-55*s, -32, -90*s);
//    activeChassis->moveToPose(-34*s, -39, 0, 1500, {.forwards=false, .minSpeed=100});
    robot::chassisGrabMogo({-24*s, -24, 50*s}, 1500, {.lead=0.1, .maxSpeed=70});
    pros::delay(1000);
    robot::chassisGrabRing({-24*s, -48, 180*s}, 2000, {.maxSpeed=100});
    pros::delay(1500);
    mogoMech.extend();
    robot::chassisMove(50, 0, 750);
//    robot::setPTO(true);
//    pros::delay(250);
//    activeChassis->moveToPose(-12*s, -12, 45*s, 5000);
//    pros::delay(500);
//    arm.moveToAngle(8);
//    pros::delay(5000);
}

void rushRush() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-55*s, -41, -90*s);
    robot::chassisGrabMogo({-24*s, -24, 60*s}, 1250, {.lead=0.1, .maxSpeed=70});
    robot::chassisGrabRing({-24*s, -48, 180*s}, 1700, {.maxSpeed=100});

    activeChassis->moveToPose(-50*s, 12, 0, 2500, {.lead=0, .maxSpeed=50});
    activeChassis->waitUntilDone();

    robot::chassisGrabMogo({-24*s, 24, 60*s}, 2000);
    robot::chassisGrabRing({-24*s, 48, 0*s}, 1250, {.minSpeed=100});

    robot::setPTO(true);

    pros::delay(1000);
    activeChassis->moveToPose(-13*s, 13, 135*s, 2000, {.forwards=false, .lead=0});
    pros::delay(500);
    mogoMech.extend();
    activeChassis->waitUntilDone();
    arm.moveToAngle(9);
    pros::delay(2000);
}

void skills() {
//    robot::chassisSetPose(-56, -8, 180);
    arm.moveToAngle(9);
    pros::delay(1000);
    robot::chassisMove(50, 0, 350);
    arm.moveToAngle(-20);
    pros::delay(1000);
    robot::chassisMove(-50, 0, 500);



//    // Mogo 1
//    robot::chassisGrabMogo({-48, 24, 30}, 5000, {.lead=0.3, .minSpeed=70});
//    pros::delay(500);
//    robot::chassisGrabRing({-60, 48, -30}, 3000);
//    robot::chassisGrabRing({-48, 48, -90}, 2000);
//    robot::chassisGrabRing({-48, 60, 0}, 2000);
//    activeChassis->moveToPose(-57, 58, -160, 3000, {.forwards=false, .lead=0});
//    activeChassis->waitUntilDone();
//    mogoMech.extend();
//    pros::delay(500);
//    robot::chassisMove(50, 0, 500);
//
//    // Mogo 2
//    activeChassis->moveToPose(-55, -8, -30, 5000, {.forwards=false, .lead=0});
//    robot::chassisGrabMogo({-48, -24, -30}, 3000, {.lead=0});
////    robot::chassisGrabRing({-24, -24, 90}, 1000);
////    robot::chassisGrabRing({-24, -48, 180}, 1000);
////    robot::chassisGrabRing({-24, -48, 180}, 1000);
////    robot::chassisGrabRing({0, -58}, 1000);
////    robot::chassisGrabRing({-48, -58}, 1000);
//    robot::chassisGrabRing({-59, -47}, 3000);
////    robot::chassisGrabRing({-58, -48}, 1000);
//    activeChassis->moveToPose(-62, -55, 50, 5000, {.forwards=false, .lead=0});
//    activeChassis->waitUntilDone();
//    mogoMech.extend();
//    activeChassis->moveToPose(54, -12, -30, 3000, {.forwards=false, .lead=0});
//    robot::chassisGrabMogo({-48, -24, -30}, 1000, {.lead=0});
}

void tunePID() {
    robot::chassisSetPose(0, 0, 0);
    activeChassis->moveToPose(0, 24, 0, 2000);
    robot::chassisPrintPose();
    activeChassis->turnToHeading(90, 2000);
    robot::chassisPrintPose();
}
}  // namespace auton
