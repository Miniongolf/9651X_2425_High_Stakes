#include "autonFuncts.hpp"

namespace auton {
void scoreWP() {
    float s = isRedAlliance ? 1 : -1;
    s = -1;
    robot::chassisSetPose(-55*s, 32, -90*s);
    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 50*s}, 900, {.lead=0.2});
    robot::chassisGrabRing({-24*s, 48, 0*s}, 2000, {.lead=0});
    activeChassis->moveToPose(-54*s, -6, 180*s, 2000);
    activeChassis->waitUntil(36);
    mogoMech.extend();
    robot::chassisGrabMogo({-24*s, -24, -60*s}, 2000, {.lead=0.2});
    robot::chassisGrabRing({-24*s, -48, 0*s}, 2000, {.lead=0, .maxSpeed=100});
    activeChassis->moveToPose(-15*s, -15, 45*s, 2000, {.maxSpeed=50});
    robot::setPTO(true);
    activeChassis->waitUntilDone();
    arm.moveToAngle(15);
    pros::delay(5000);
}

void scoreMax() {
    conveyor.resetIndexQueue();
    float s = isRedAlliance ? 1 : -1;
    s = 1;
    robot::chassisSetPose(-55*s, 41, -90*s);
    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 160*s}, 1000, {.lead=0.1, .maxSpeed=100});
    robot::chassisStop();
    pros::delay(600);
    // Ring 1
    robot::chassisGrabRing({-7*s, 43, 80*s}, 1750, {.lead=0, .maxSpeed=100});
    // Ring 2
    activeChassis->moveToPose(-18*s, 24, 0*s, 1500, {.forwards=false});
    robot::chassisGrabRing({-24*s, 48, 0*s}, 1750, {.lead=0.1, .maxSpeed=100});
    pros::delay(500);
    robot::setPTO(true);
    pros::delay(1000);
    arm.moveToAngle(8);
    activeChassis->moveToPose(-17, 17, 135, 2000, {.lead=0, .minSpeed=70});
    activeChassis->waitUntil(10);
    mogoMech.extend();
    pros::delay(5000);
    // Ring 3
//    robot::chassisGrabRing({-29*s, 52, -60*s}, 2000, {.lead=0, .maxSpeed=100}, true, 1250);
//    activeChassis->turnToPoint(-24*s, 48, 1000, {.forwards=false});
    //    robot::setPTO(true);
    //    // Alliance Stake
    //    activeChassis->moveToPose(-60*s, 10, -103*s, 4000, {.forwards=true});
    //    pros::delay(1500);
    //    conveyor.forwards();
    //    arm.moveToAngle(9);
    //    activeChassis->waitUntil(20);
    //    conveyor.idle();
    //    hooks.move(0);
    //    activeChassis->waitUntilDone();
    //
    //    arm.moveToAngle(-10);
    //    pros::delay(750);
    //    robot::chassisMove(-127, 0, 300);
    //    // Bar touch
    //    activeChassis->turnToHeading(90*s, 1250);
    //    activeChassis->waitUntilDone();
    //    robot::chassisPrintPose();
    //    robot::chassisMove(50, 0, 1000);
    //    pros::delay(2000);
}

void scoreRush() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-55*s, 16, -90*s);
    robot::chassisGrabMogo({0, -48, -60*s}, 2000, {.minSpeed=100});
    robot::chassisPrintPose();
}

void rushWP() {
    float s = isRedAlliance ? 1 : -1;
    s = 1;
    robot::chassisSetPose(-55*s, -32, -90*s);
//    activeChassis->moveToPose(-34*s, -39, 0, 1500, {.forwards=false, .minSpeed=100});
    robot::chassisGrabMogo({-24*s, -24, 50*s}, 1500, {.lead=0.1, .maxSpeed=70});
    pros::delay(1000);
    robot::chassisGrabRing({-24*s, -48, 180*s}, 2000, {.lead=0, .maxSpeed=100});
    pros::delay(1500);
    mogoMech.extend();
    robot::chassisMove(-50, 0, 1000);
    robot::setPTO(true);
    pros::delay(250);
    activeChassis->moveToPose(-12*s, -12, 45*s, 5000);
    pros::delay(500);
    arm.moveToAngle(8);
    pros::delay(5000);
}

void rushRush() {
    float s = isRedAlliance ? 1 : -1;
    s = 1;
    robot::chassisSetPose(-55*s, -41, -90*s);
    robot::chassisGrabMogo({-24*s, -24, 60*s}, 1250, {.lead=0.1, .maxSpeed=70});
    robot::chassisGrabRing({-24*s, -48, 180*s}, 1700, {.lead=0, .maxSpeed=100});

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
    robot::chassisSetPose(-51, 0, -90);

    // Alliance Stake
    arm.moveToAngle(9);
    pros::delay(500);
    activeChassis->moveToPose(-55.5, 0, -90, 1000);
    activeChassis->waitUntilDone();
    arm.moveToAngle(-20);
    pros::delay(750);
    robot::chassisMove(-50, -50, 500);

    // Mogo 1
    robot::chassisGrabMogo({-48, 24, 60}, 2000, {.lead=0, .maxSpeed=100});
    pros::delay(500);
    robot::setPTO(false);
    robot::chassisGrabRing({-24, 24, 80}, 1000);
    robot::chassisGrabRing({-24, 48, 0}, 1000);
    robot::chassisGrabRing({0, 56, 60}, 1000);
    robot::chassisGrabRing({-48, 60, -90}, 1000);
    robot::chassisGrabRing({-48, 48, 180}, 1000);
    robot::chassisGrabRing({-60, 48, -90}, 1000);
    activeChassis->moveToPose(-68, 58, -20, 1000, {.forwards=false, .lead=0});
    activeChassis->waitUntilDone();
    mogoMech.extend();

    // Mogo 2
    activeChassis->moveToPose(-55, -8, -30, 3000, {.lead=0});
    robot::chassisGrabMogo({-48, -24, -30}, 1000, {.lead=0});
    robot::chassisGrabRing({-24, -24, 90}, 1000);
    robot::chassisGrabRing({-24, -48, 180}, 1000);
    robot::chassisGrabRing({-24, -48, 180}, 1000);
    robot::chassisGrabRing({0, -58}, 1000);
    robot::chassisGrabRing({-48, -58}, 1000);
    robot::chassisGrabRing({-48, -48}, 1000);
    robot::chassisGrabRing({-58, -48}, 1000);
    activeChassis->moveToPose(-62, -55, 30, 3000, {.forwards=false, .lead=0});
    activeChassis->waitUntilDone();
    mogoMech.extend();
}

void tunePID() {
    robot::chassisSetPose(0, 0, 0);
    activeChassis->moveToPose(0, 24, 0, 2000);
    robot::chassisPrintPose();
    activeChassis->turnToHeading(90, 2000);
    robot::chassisPrintPose();
}
}  // namespace auton
