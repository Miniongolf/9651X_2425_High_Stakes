#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/arm/arm.hpp"
#include "robot/subsys/intake/intake.hpp"

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

void auton::skills() {
    // Set colour sort to red alliance
    intake.setAlliance(Alliance::RED);

    // Start pose (in front of alliance stake)
    chassis.setPose(-64, 0, 90);
    intake.forwards();
    pros::delay(700);
    intake.idle();

    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveTimed(50, -20, 500);
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed=50});
    chassis.moveToPoint(-24, 24, 1000); // Ring 1 (before mogo)

    // Grab first mogo (top left corner)
    robot::safeGrabMogo(-48, 24, 1000);

    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.moveToPose(-24, 30, 65, 800, {.minSpeed = 70}, false);
    chassis.moveToPoint(24, 47, 1250, {.minSpeed = 100, .earlyExitRange = 15}, false); // Ring 2
    chassis.moveToPoint(42, 58, 750, {.minSpeed = 90}, false); // Ring 3
    chassis.safeMoveToPoint(24, 47, 1000, {.forwards = false}, false); // Ring 4
    chassis.safeMoveToPoint(-24, 47, 800, 2000, {.minSpeed = 0}, {.minSpeed = 110, .earlyExitRange = 24}, false); // Ring 5
    chassis.moveToPoint(-43, 47, 2000,   {.maxSpeed = 40}, false);
    chassis.moveToPoint(-48, 40, 2000,   {.maxSpeed = 40}, false); // Ring 6
    chassis.swingToPoint(-48, 60, lemlib::DriveSide::RIGHT, 1000, {}, false);
    chassis.safeMoveToPoint(-48, 60, 1000, {}, false);
    intake.setMode(Intake::modes::INDEX);
    // chassis.moveTimed(70, 0, 500, false); // Grab top wallstake 1

    // Drop goal in corner
    chassis.turnToPoint(-70, 70, 600, {.forwards = false});
    chassis.moveTimed(-50, 0, 600, false);
    mogoMech.release(true);
    // Top wallstake
    chassis.safeMoveToPoint(-12, 48, 500, 1000);
    chassis.swingToPoint(0, 90, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 70}, false);
    chassis.moveTimed(70, 0, 500, false);
    chassis.moveTimed(-50, 0, 300);
    pros::delay(50);
    arm.moveToPosition(Arm::wall);
    pros::delay(500);
    chassis.brake();
    intake.idle();
    chassis.moveTimed(50, 0, 1000, false);
    robot::scoreWallStake(true, false);
    chassis.setPose(0, 55, 0);

    // Push top right mogo
    chassis.swingToPoint(54, 18, lemlib::DriveSide::RIGHT, 750, {}, false);
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    arm.moveToPosition(Arm::idle);
    chassis.safeMoveToPoint(36, 29, 1000, {.forwards = false});
    chassis.turnToPoint(47, 16, 1000);
    intake.idle();
    chassis.swingToPoint(58, 22, lemlib::DriveSide::LEFT, 700);
    chassis.moveToPoint(65, 65, 2000);
    robot::safeGrabMogo(48, 0, 1500);
    intake.setMode(Intake::modes::INDEX);
    chassis.safeMoveToPoint(36, 24, 1000, {.forwards = false});
    chassis.swingToPoint(0, 0, lemlib::DriveSide::LEFT, 750);
    chassis.moveToPoint(0, 0, 1000, {}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    pros::delay(1000);
    intake.idle();
    chassis.moveToPoint(-42, -42, 1000);
    intake.forwards();
    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 600);
    chassis.moveTimed(50, 0, 750);
    chassis.turnToPoint(-70, -70, 750, {.forwards = false});
    chassis.moveTimed(-50, 0, 750);
    intake.idle();
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.swingToPoint(-60, -48, lemlib::DriveSide::LEFT, 750);
}