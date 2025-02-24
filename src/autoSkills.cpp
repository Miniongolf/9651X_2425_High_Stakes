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
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveToPose(-24, 24, 30, 1000); // Ring 1
    // Grab first mogo
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.swingToPoint(24, 48, lemlib::DriveSide::LEFT, 1000, {.minSpeed=127});
    chassis.moveToPoint(24, 48, 1000, {}, false);
    chassis.moveToPoint(48, 60, 1000);
    chassis.moveToPoint(24, 48, 1000, {.forwards=false});
    chassis.chainTurnToHeading(-90, false, true, 1000);
    chassis.moveToPoint(-44, 48, 3000, {.earlyExitRange=24});
    chassis.moveToPoint(-44, 48, 3000, {.maxSpeed=60}, false);
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    chassis.safeMoveToPoint(-48, 60, 1000);
    // Drop goal in corner
    chassis.swingToHeading(110, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    mogoMech.release();
    chassis.moveToPoint(0, 60, 1000);
}