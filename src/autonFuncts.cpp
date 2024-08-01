#include "autonFuncts.hpp"

namespace auton {
void scoreWP() {
    conveyor.resetIndexQueue();
    float s = isRedAlliance ? 1 : -1;
    s = -1;
    robot::chassisSetPose(-55*s, 30, -90*s);
    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 60*s}, 800, {.lead=0});
    pros::delay(150);
    // Ring 1
    robot::chassisGrabRing({-7*s, 40, 70*s}, 1500, {.lead=0, .minSpeed=70});
    // Ring 2
    robot::chassisGrabRing({-1*s, 60, 80*s}, 1000, {.lead=0, .maxSpeed=100});
//    pros::delay(500);
//    intake.move(0);
//    activeChassis->moveToPose(-7*s, 34, -45*s, 1500, {.forwards=false});
    // Ring 3
    pros::delay(1000);
    conveyor.queueIndex(1);
//    activeChassis->moveToPose(-6*s, 40, -80*s, 1000, {.forwards=false});
    activeChassis->turnToPoint(-24*s, 48, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed=90});
    robot::chassisGrabRing({-29*s, 52, -60*s}, 2000, {.lead=0, .maxSpeed=100});
    robot::setPTO(true);
    // Alliance Stake
    activeChassis->moveToPose(-60*s, 4, -103*s, 4000, {.forwards=true});
    pros::delay(1500);
    conveyor.forwards();
    arm.moveToAngle(9);
    activeChassis->waitUntilDone();
    arm.moveToAngle(-10);
    pros::delay(750);
    robot::chassisMove(-127, 0, 300);
    // Bar touch
    activeChassis->turnToHeading(90*s, 1250);
    activeChassis->waitUntilDone();
    robot::chassisPrintPose();
    robot::chassisMove(50, 0, 1000);
    pros::delay(2000);
}

void scoreMax() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-50*s, 10.5, -137*s);
    arm.moveToAngle(10);
    pros::delay(500);
    robot::chassisMove(60, 0, 600);
    arm.moveToAngle(-20);
    pros::delay(750);
    robot::setPTO(false);
    robot::chassisMove(-50, 0, 750);

    // Mogo
    robot::chassisGrabMogo({-24*s, 24, 60*s}, 2000, {.lead=0.1, .maxSpeed=100});

    // Close ring
    robot::chassisGrabRing({-30*s, 48, 0*s}, 2000, {.lead=0, .maxSpeed=100});
    activeChassis->moveToPose(-35*s, 35, 45*s, 2000, {.forwards=false});
    pros::delay(500);

    // Mid ring 1
    robot::chassisGrabRing({-12*s, 45, 85*s}, 2000, {.lead=0});
    pros::delay(500);
    activeChassis->moveToPose(-24*s, 48, 90*s, 1000, {.forwards=false});
    pros::delay(500);

    // Mid ring 2
    robot::chassisGrabRing({-12*s, 51, 90*s}, 2000);
    robot::chassisMove(50, 0, 200);
    pros::delay(1000);
    conveyor.stop();

    // Bar touch
    robot::setPTO(true);
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

void rushRush() {
    float s = isRedAlliance ? 1 : -1;
    robot::chassisSetPose(-56*s, -31.5, -90*s);
    robot::chassisGrabMogo({0, -48, -60*s}, 2000, {.minSpeed=100});
    robot::chassisPrintPose();
}

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
