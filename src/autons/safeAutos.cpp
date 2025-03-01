#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/helperFuncts.hpp"

void auton::twoRing(bool positive) {
    resetSplit();
    int s = (robotAlliance == Alliance::RED) && positive ? 1 : -1;
    int p = positive ? 1 : -1;
    chassis.setPose(-50 * s, -24 * p, -90 * s);
    // Grab mogo
    robot::safeGrabMogo(-24 * s, -24 * p, 1000);
    // Grab ring
    chassis.turnToPoint(-24 * s, -48 * p, 600, {}, false);
    intake.forwards();
    chassis.moveToPoint(-24 * s, -42 * p, 750);
    if (robotAlliance == Alliance::RED) {
        if (positive) {
            chassis.safeMoveToPoint(-38, -60, 1000, {}, false);
            chassis.turnToPoint(-70, -70, 750);
            doinker.extend();
            chassis.moveTimed(60, 0, 300, false);
            chassis.moveTimed(50, 0, 1000);
            chassis.turnToHeading(0, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
            chassis.turnToPoint(-160, -50, 800);
            doinker.retract();
            pros::delay(100);
            chassis.turnToPoint(-70, -70, 750);
            chassis.moveTimed(60, 0, 750, false);
            pros::delay(500);
            chassis.moveTimed(-50, 0, 700, false);
        }
    }
    pros::delay(500);
    arm.moveToPosition(Arm::wall);
    chassis.turnToPoint(-16 * s, -16 * s, 750, {}, false);
    chassis.moveToPoint(-16 * s, -16 * s, 2000, {.maxSpeed = 60}, false);
    chassis.brake();
    nextSplit("Touch bar");
}

void auton::safeAWP(bool positive) {
    resetSplit();
    int s = (robotAlliance == Alliance::RED) ? 1 : -1;
    int p = positive ? 1 : -1;
    chassis.setPose(-55 * s, -13 * p, 0 + 90 * (p - 1));
    // Alliance stake
    chassis.moveToPoint(-55 * s, 9.5 * p, 750, {.maxSpeed = 80});
    chassis.swingToHeading(90 * s, lemlib::DriveSide::LEFT * s * p, 800, {.minSpeed = 0});
    chassis.moveTimed(-70, 0, 500);
    robot::scoreAllianceStake();

    // Grab mogo
    chassis.moveTimed(50, 20 * s, 300);
    chassis.turnToPoint(-24*s, -24*p, 750, {.forwards = false});
    chassis.moveToPoint(-24*s, -24*p, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 24}, false);
    chassis.brake();
    chassis.moveToPoint(-24*s, -24*p, 1000, {.forwards = false, .maxSpeed = 50}, false);
    mogoMech.clamp();

    // Grab ring
    intake.forwards();
    chassis.safeMoveToPoint(-27 * s, -42 * p, 1000);
    chassis.moveTimed(-50, 0, 100);

    // Corner
    if (robotAlliance == Alliance::RED) {
        if (positive) {
            chassis.safeMoveToPoint(-60, -30, 1000, {.forwards = false});
            chassis.turnToPoint(-70, -70, 1000, {}, false);
            chassis.moveTimed(50, 0, 500, false);
            chassis.brake();
            doinker.extend();
            chassis.moveTimed(30, 0, 500);
        } else {
            chassis.safeMoveToPoint(-11, 45, 1000);
            chassis.moveTimed(-60, -30, 500);
            chassis.safeMoveToPoint(-11, 41, 1000, {.maxSpeed = 60});
            chassis.moveToPoint(-36, 36, 1000, {.forwards = false}, false);
            // chassis.safeMoveToPoint(-55, 55, 1000);
            // chassis.turnToHeading(-45, 750);
            // chassis.moveTimed(80, 0, 1000, false);
            // chassis.moveTimed(-50, 0, 300, false);
            // pros::delay(500);
        }
    } else { // Blue alliance
        if (positive) {
            chassis.safeMoveToPoint(60, -30, 1000, {.forwards = false});
            chassis.turnToPoint(70, -70, 1000, {}, false);
            chassis.moveTimed(50, 0, 500, false);
            chassis.brake();
            doinker.extend();
            chassis.moveTimed(30, 0, 500, false);
            chassis.turnToHeading(-90, 750, {.minSpeed = 127}, false);
            doinker.retract();
        } else {
            chassis.moveTimed(-50, -50, 700);
            chassis.safeMoveToPoint(20, 37, 1500, {.maxSpeed = 60}, false);
            chassis.moveTimed(-60, -30, 500, false);
            chassis.safeMoveToPoint(20, 42, 1500, {.maxSpeed = 60}, false);
            pros::delay(300);
            chassis.moveToPoint(36, 36, 1000, {.forwards = false}, false);
        }
    }
    arm.moveToPosition(Arm::wall);
    chassis.turnToPoint(0, 0, 1000, {}, true);
    pros::delay(1000);
    intake.idle();
    // pros::delay(300);
    chassis.moveToPoint(-16 * s, -16 * p, 1500, {.maxSpeed = 60}, false);
    robot::printPose();
    nextSplit("Touch bar");
}
