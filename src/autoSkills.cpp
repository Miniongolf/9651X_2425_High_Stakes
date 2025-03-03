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

    // chassis.setPose(5, 61, 0); // Odom reset on top wallstake, don't preserve imu heading
    pros::delay(10);
    nextSplit("Top wallstake");

    // Top right mogo
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    arm.moveToPosition(60);
    chassis.moveTimed(-60, 0, 500, false);
    arm.moveToPosition(Arm::idle);
    chassis.brake();
    // Grab rings
    intake.forwards();
    chassis.turnToPoint(46, 46, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(46, 46, 1000, {}, false);
    intake.setMode(Intake::modes::INDEX);
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
    robot::safeGrabMogo(49, -15, 2500);
    robot::printPose();
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    nextSplit("Third Goal");

    // Cross field diagonally
    chassis.turnToPoint(36, 28, 600, {.minSpeed = 0});
    chassis.moveToPoint(36, 28, 1000, {.maxSpeed = 80, .minSpeed = 0}, false);
    chassis.brake();
    chassis.turnToPoint(0, 0, 1000, {}, false);
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-24, -24, 1000, {.maxSpeed = 70}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.moveToPoint(-46, -46, 1000);
    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 800, {}, false);
    chassis.moveTimed(50, 0, 750, false);
    chassis.turnToHeading(-45, 750, {.minSpeed = 50, .earlyExitRange = 5}, false);
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 400, {}, false);
    chassis.moveTimed(50, 0, 500);
    // Drop in corner
    chassis.turnToPoint(-70, -70, 750, {.forwards = false});
    chassis.moveTimed(-50, 0, 750, false);
    intake.idle();
    mogoMech.release(true);
    nextSplit("Cross-court mogo");

    // Grab 4th goal
    chassis.moveTimed(50, 0, 500, false);
    robot::safeGrabMogo(-45, -17, 1000);
    // Fill goal
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.safeMoveToPoint(-24, -48, 1000);
    chassis.safeMoveToPoint(0, -54, 1000);
    chassis.safeMoveToPoint(46, -60, 1000);
    chassis.safeMoveToPoint(48, -48, 1000);
    chassis.safeMoveToPoint(24, -48, 1000, false);
    chassis.safeMoveToPoint(55, -55, 1000, {.forwards = false}, false);
    mogoMech.release(true);
    intake.idle();
    chassis.moveTimed(-50, 0, 500);
    chassis.safeMoveToPoint(24, -24, 1000, false);
    chassis.turnToPoint(0, 0, 750);
    arm.moveToPosition(Arm::hang);
    chassis.moveToPoint(20, -20, 1000, {}, false);
    chassis.brake();
    chassis.moveTimed(90, 0, 2000, false);
    arm.moveToPosition(Arm::idle);
    pros::delay(1500);
    arm.moveToPosition(Arm::hang);
    nextSplit("Hang");
    pros::delay(2000);
}