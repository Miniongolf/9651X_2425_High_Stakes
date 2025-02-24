#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

bool headingCloseTo(double target) {
    return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5;
};

void auton::skills() {
    // Disable colour sort
    robotAlliance = Alliance::NONE;

    // Start pose (in front of alliance stake)
    chassis.setPose(-64, 0, 90);
    intake.forwards();
    pros::delay(500);
    intake.idle();

    // Ring before mogo
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.moveTimed(50, 0, 500);
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed=127});
    chassis.moveToPoint(-24, 24, 1000);
    // Grab first mogo
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.moveToPoint(-24, 24, 1000);
    chassis.safeMoveToPoint(24, 48, 1250, {.minSpeed=100}, false);
    chassis.moveToPoint(48, 60, 1000, {.maxSpeed=70});
    chassis.safeMoveToPoint(24, 48, 1000, {.forwards=false});
    chassis.safeMoveToPoint(-60, 48, 3000, {.maxSpeed=70}, false);
    pros::delay(1500);
    intake.setMode(Intake::modes::INDEX);
    chassis.turnToPoint(-48, 60, 1000);
    chassis.moveToPoint(-48, 60, 1000, {}, false);
    // Drop goal in corner
    chassis.moveTimed(-50, 0, 500, false);
    mogoMech.release();
    chassis.moveToPoint(5, 60, 1500);
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 1000);
    arm.moveToPosition(Arm::wall);
    intake.idle();
    intake.setMode(Intake::modes::HOLD);
    chassis.moveTimed(50, 0, 750, false);
    intake.reverse();
}