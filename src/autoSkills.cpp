#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/arm/arm.hpp"
#include "robot/subsys/intake/intake.hpp"
#include <string>

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

bool robotCloseTo(lemlib::Pose target) {
    return target.distance(chassis.getPose()) <= 5 &&
           std::fabs(lemlib::angleError(target.theta, chassis.getPose().theta, false)) <= 5;
}

void auton::skills() {
    startTime = from_msec(pros::millis());
    // Set colour sort to red alliance
    intake.setAlliance(Alliance::RED);

    // Start pose (in front of alliance stake)
    chassis.setPose(-64, 0, 90);
    intake.forwards();
    pros::delay(700);
    intake.idle();

    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.moveTimed(50, -20, 500);
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed = 50});
    chassis.moveToPoint(-24, 24, 750); // Ring 1 (before mogo)

    // Grab first mogo (top left corner)
    robot::safeGrabMogo(-48, 24, 1000);

    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.moveToPose(-0, 39, 55, 800, {.minSpeed = 60}, false);
    chassis.moveToPoint(24, 47, 1250, {.minSpeed = 90, .earlyExitRange = 15}, false); // Ring 2
    chassis.moveToPoint(40, 55, 750, {.maxSpeed = 90, .minSpeed = 70}, false); // Ring 3
    chassis.moveToPoint(24, 48, 1000, {.forwards = false}, false); // Ring 4
    chassis.safeMoveToPoint(-24, 48, 800, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 0},
                            {.minSpeed = 110, .earlyExitRange = 24},
                            false); // Ring 5
    chassis.moveToPoint(-43, 48, 2000, {.maxSpeed = 40}, false);
    chassis.moveToPoint(-48, 42, 2000, {.maxSpeed = 40}, false); // Ring 6
    chassis.swingToPoint(-48, 60, lemlib::DriveSide::RIGHT, 1000, {}, false);
    chassis.safeMoveToPoint(-48, 60, 1000, {}, false);
    intake.setMode(Intake::modes::INDEX);
    
    // Drop first goal in corner
    chassis.turnToPoint(-70, 70, 600, {.forwards = false});
    chassis.moveTimed(-50, 0, 600, false);
    mogoMech.release(true);
    nextSplit("First corner");
    // Top wallstake
    chassis.safeMoveToPoint(-15, 48, 500, 1000);
    chassis.swingToPoint(0, 150, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 70}, false);
    chassis.moveTimed(50, 0, 500, false);
    pros::delay(100);
    chassis.moveTimed(-50, 0, 200);
    chassis.brake();
    pros::delay(500);
    arm.moveToPosition(Arm::wall);
    intake.idle();
    pros::delay(600);
    robot::scoreWallStake(true, true);
    robot::printPose();

    // chassis.setPose(0, 61, 0); // Odom reset on top wallstake, don't preserve imu heading
    pros::delay(10);
    nextSplit("Top wallstake");

    // Top right mogo
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    arm.moveToPosition(60);
    chassis.moveTimed(-60, 0, 500, false);
    arm.moveToPosition(Arm::idle);
    chassis.brake();
    // Grab ring for alliance stake
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.turnToPoint(46, 46, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(46, 46, 1000, {}, false);
    pros::delay(300);
    // Go to push goal
    chassis.safeMoveToPoint(46, 29, 600, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, {}, false);
    chassis.brake();
    pros::delay(50);
    // Push goal into corner
    chassis.swingToPoint(90, 70, lemlib::DriveSide::LEFT, 1000, {}, false);
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    chassis.moveTimed(100, 0, 800, false);
    chassis.moveTimed(50, 0, 500, false);
    chassis.moveTimed(-50, 0, 300, false);
    pros::delay(200);
    chassis.moveTimed(100, 0, 350, false);
    intake.idle();
    nextSplit("Corner Mogo");

    // Grab 3rd goal
    robot::safeGrabMogo(51, -8, 2500);
    robot::printPose();
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle();
    nextSplit("Third Goal");

    // Cross field diagonally
    chassis.turnToPoint(36, 36, 600, {.minSpeed = 0});
    chassis.moveToPoint(36, 36, 1000, {.maxSpeed = 80, .minSpeed = 0}, false);
    chassis.brake();
    chassis.turnToPoint(0, 0, 1000, {}, false);
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-24, -24, 1000, {.maxSpeed = 70}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.moveToPoint(-40, -39, 1000);
    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 800, {}, false);
    chassis.moveTimed(50, 0, 750, false);
    chassis.turnToHeading(-45, 750, {.minSpeed = 50, .earlyExitRange = 5}, false);
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 400, {}, false);
    // Drop in corner
    chassis.turnToPoint(-70, -70, 750, {.forwards = false});
    chassis.moveTimed(-50, 0, 750, false);
    intake.idle();
    mogoMech.release(true);
    nextSplit("Cross-court mogo");

    // Grab 4th goal
    chassis.moveTimed(50, 0, 500, false);
    robot::safeGrabMogo(-48, -24, 1000);
    // Bottom wall stake
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.safeMoveToPoint(-24, -48, 1000, {.minSpeed = 0}, false);
    chassis.moveToPoint(-6, -51, 1000, {.maxSpeed = 70}, false);
    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 500, {}, false);
    chassis.moveTimed(70, 0, 500, false);
    chassis.moveTimed(-50, 0, 300);
    pros::delay(50);
    arm.moveToPosition(Arm::wall);
    chassis.brake();
    robot::scoreWallStake(true, true);
    chassis.setPose(0, -61, 0); // Odom reset on bottom wallstake
    nextSplit("Bottom wallstake");

    chassis.moveTimed(-50, 0, 500, false);
    arm.moveToPosition(Arm::idle);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.safeMoveToPoint(24, -24, 1000, {}, false);
    chassis.turnToHeading(-150, 750);
    chassis.moveToPoint(24, -47, 1000, {}, false);
    chassis.safeMoveToPoint(43, -55, 1000);
    chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 600);
    chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 600);

    // Place last mogo in corner
    chassis.turnToPoint(70, -70, 750);
    chassis.moveTimed(50, 0, 750, false);
    intake.idle();
    pros::delay(100);
    chassis.turnToHeading(-45, 750);
    chassis.moveTimed(-50, 0, 750);
    mogoMech.release(true);
    nextSplit("Last mogo");

    // Hang
    chassis.moveTimed(70, 40, 500);
    intake.forwards();
    chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 750, {.minSpeed = 100});
    chassis.turnToHeading(-45, 750, {}, false);
    arm.moveToPosition(Arm::hang);
    intake.idle();
    chassis.moveTimed(60, 0, 1000);
    arm.moveToPosition(Arm::idle);
    pros::delay(500);
    arm.moveToPosition(Arm::hang);
    nextSplit("Hang");
}