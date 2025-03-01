#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/helperFuncts.hpp"

void auton::twoRing(bool positive) {
    resetSplit();
    int s = ((robotAlliance == Alliance::RED) && positive) ? 1 : -1;
    chassis.setPose(-50 * s, -24, -90 * s);
    // Grab mogo
    robot::safeGrabMogo(-24 * s, -24, 1000);
    // Grab ring
    chassis.turnToPoint(-24 * s, -48, 600, {}, false);
    intake.forwards();
    chassis.moveToPoint(-24 * s, -42, 750);
    if (robotAlliance == Alliance::RED) {
        chassis.safeMoveToPoint(-38, -60, 1000, {}, false);
        chassis.turnToPoint(-70, -70, 750);
        doinker.extend();
        chassis.moveTimed(60, 0, 300, false);
        chassis.moveTimed(50, 0, 1000);
        chassis.turnToHeading(0, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis.turnToPoint(-60, -50, 800);
        doinker.retract();
        pros::delay(100);
        chassis.turnToPoint(-70, -70, 750);
        chassis.moveTimed(60, 0, 750, false);
        pros::delay(500);
        chassis.moveTimed(-50, 0, 700, false);
    }
    pros::delay(500);
    arm.moveToPosition(Arm::wall);
    chassis.turnToPoint(-12 * s, -12, 750, {}, false);
    chassis.moveToPoint(-12 * s, -12, 2000, {.maxSpeed = 60}, false);
    chassis.brake();
    nextSplit("Touch bar");
}

void auton::safeAWP(bool positive) {
    resetSplit();
    int s = ((robotAlliance == Alliance::RED) && positive) ? 1 : -1;
    chassis.setPose(-55 * s, -13, 0);
    // Alliance stake
    chassis.moveToPoint(-55 * s, 9.5, 750, {.maxSpeed = 80});
    chassis.swingToHeading(90 * s, lemlib::DriveSide::LEFT * s, 800);
    chassis.moveTimed(-70, 0, 500);
    robot::scoreAllianceStake();

    // Grab mogo
    chassis.moveTimed(50, 20 * s, 300);
    robot::safeGrabMogo(-24 * s, -24, 1250);

    // Grab ring
    intake.forwards();
    chassis.safeMoveToPoint(-27 * s, -45, 1000);
    chassis.moveTimed(-50, 0, 100);
    
    // Corner
    if (robotAlliance == Alliance::RED) {
        chassis.safeMoveToPoint(-60 * s, -30, 1000, {.forwards = false});
        chassis.turnToPoint(-70 * s, -70, 1000, {}, false);
        chassis.moveTimed(50, 0, 500, false);
        chassis.brake();
        doinker.extend();
        chassis.moveTimed(30, 0, 500);
    } else {
        chassis.safeMoveToPoint(-60 * s, -30, 1000, {.forwards = false});
        chassis.turnToPoint(-70 * s, -70, 1000, {}, false);
        chassis.moveTimed(50, 0, 500, false);
        chassis.brake();
        doinker.extend();
        chassis.moveTimed(30, 0, 500);
    }
    chassis.turnToPoint(-16, -16, 1000, {}, false);
    arm.moveToPosition(Arm::wall);
    intake.idle();
    // pros::delay(300);
    chassis.moveToPoint(-16 * s, -16, 1500, {.maxSpeed = 60}, false);
    robot::printPose();
    nextSplit("Touch bar");
}
