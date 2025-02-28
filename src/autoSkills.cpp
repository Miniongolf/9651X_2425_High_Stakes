#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/arm/arm.hpp"
#include "robot/subsys/intake/intake.hpp"
#include "units/units.hpp"
#include <string>

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

int split = 0;
Time startTime = 0_msec;

void nextSplit(std::string name) {
    split++;
    Time elapsedTime = from_msec(pros::millis()) - startTime;
    std::cout << "Split " << split << ": " << name << std::endl;
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

    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveTimed(50, -20, 500);
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed = 50});
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
    chassis.safeMoveToPoint(-24, 47, 800, 2000, {.minSpeed = 0}, {.minSpeed = 110, .earlyExitRange = 24},
                            false); // Ring 5
    chassis.moveToPoint(-43, 47, 2000, {.maxSpeed = 40}, false);
    chassis.moveToPoint(-48, 40, 2000, {.maxSpeed = 40}, false); // Ring 6
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
    chassis.setPose(0, 55, 0); // Odom reset on top wallstake
    nextSplit("Top wallstake");

    // Top right mogo
    chassis.swingToPoint(39, 51, lemlib::DriveSide::LEFT, 750, {.minSpeed = 50}, false);
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    arm.moveToPosition(Arm::idle);
    // Grab ring for alliance stake
    chassis.safeMoveToPoint(39, 51, 1000, {.forwards = false});
    chassis.swingToPoint(45, 20, lemlib::DriveSide::RIGHT, 650, {}, false);
    chassis.moveToPoint(45, 20, 1000, {}, false);
    // Push goal into corner
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    chassis.swingToPoint(70, 53, lemlib::DriveSide::LEFT, 600, {}, false);
    chassis.moveTimed(80, 0, 1000, false);
    intake.idle();
    nextSplit("Corner Mogo");

    // Grab 3rd goal + alliance stake
    robot::safeGrabMogo(48, 0, 1500);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle();
    arm.moveToPosition(Arm::wall);
    chassis.turnToHeading(90, 750, {.minSpeed = 0}, false);
    pros::delay(500);
    chassis.moveTimed(50, 0, 600, false);
    robot::scoreWallStake(true, false);
    chassis.setPose(55, 0, 90); // Odom reset on blue alliance stake
    nextSplit("Alliance stake");

    // Cross field diagonally
    chassis.moveTimed(-50, 30, 500);

    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE},
                           false);
    arm.moveToPosition(Arm::idle);
    intake.setMode(Intake::modes::INDEX);
    chassis.safeMoveToPoint(36, 24, 1000, {}, false);
    intake.forwards(); // Start crossing
    chassis.swingToPoint(0, 0, lemlib::DriveSide::LEFT, 750, {.minSpeed = 0}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    chassis.moveToPoint(0, 0, 1000, {.minSpeed = 70}, false);
    chassis.moveToPoint(-24, -24, 1000, {.minSpeed = 70}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.moveToPoint(-40, -39, 1000);
    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 600, {}, false);
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
    pros::delay(500);
    chassis.brake();
    intake.idle();
    chassis.moveTimed(50, 0, 1000, false);
    robot::scoreWallStake(true, false);
    chassis.setPose(0, -55, 0); // Odom reset on bottom wallstake
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
    chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 750, {.minSpeed=100});
    chassis.turnToHeading(-45, 750, {}, false);
    arm.moveToPosition(Arm::hang);
    intake.idle();
    chassis.moveTimed(60, 0, 1000);
    arm.moveToPosition(Arm::idle);
    pros::delay(500);
    arm.moveToPosition(Arm::hang);
    nextSplit("Hang");
}