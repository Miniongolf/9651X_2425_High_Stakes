#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

void auton::skills() {
    // Set colour sort to red alliance
    intake.setAlliance(Alliance::RED);

    // Start pose (in front of alliance stake)
    chassis.setPose(-64, 0, 90);
    intake.forwards();
    pros::delay(500);
    intake.idle();

    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.moveTimed(50, -20, 500);
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.maxSpeed = 80, .minSpeed = 30});
    chassis.moveToPoint(-24, 24, 1000); // Ring 1 (before mogo)
    // Grab first mogo (top left corner)
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    // Fill mogo
    chassis.moveToPose(-24, 30, 65, 800, {.minSpeed = 70});
    chassis.moveToPoint(24, 45, 1250, {.minSpeed = 100, .earlyExitRange = 15}); // Ring 2
    chassis.moveToPoint(42, 58, 750, {.minSpeed = 90}); // Ring 3
    chassis.safeMoveToPoint(24, 46, 1000, {.forwards = false}); // Ring 4
    chassis.safeMoveToPoint(-24, 46, 800, 2000, {.minSpeed = 0}, {.minSpeed = 110, .earlyExitRange = 24}); // Ring 5
    chassis.moveToPoint(-60, 46, 2000, {.maxSpeed = 45}, false); // Ring 6
    pros::delay(300);
    chassis.swingToPoint(-48, 60, lemlib::DriveSide::LEFT, 750, {}, false);
    intake.setMode(Intake::modes::INDEX);
    chassis.moveToPoint(-48, 60, 1000, {.maxSpeed = 80}, false); // Grab top wallstake 1
    // Drop goal in corner
    chassis.swingToPoint(-70, 70, lemlib::DriveSide::LEFT, 600, {.forwards = false});
    chassis.moveTimed(-50, 0, 300, false);
    mogoMech.release(true);
    // Top wallstake
    chassis.safeMoveToPoint(-12, 48, 500, 1000);
    chassis.swingToPoint(0, 90, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 70}, false);
    chassis.moveTimed(70, 0, 750, false);
    pros::delay(500);
    arm.moveToPosition(Arm::wall);
    chassis.moveTimed(-50, 0, 200);
    chassis.brake();
    intake.idle();
    chassis.turnToPoint(0, 70, 1000, {.minSpeed = 35});
    chassis.moveTimed(50, 0, 1000, false);
    robot::scoreWallStake(true, false);

    // Second mogo (bottom left)
    chassis.swingToPoint(-48, 0, lemlib::DriveSide::LEFT, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-48, 0, 1000, {.forwards = false});
    intake.idle();
    pros::delay(300);
    arm.moveToPosition(Arm::idle);
    robot::safeGrabMogo(-48, -24, 1000); // Grab second goal
    intake.forwards();
    chassis.swingToPoint(-24, -24, lemlib::DriveSide::RIGHT, 1000);
    chassis.moveToPoint(-24, -24, 1000, {.minSpeed = 127});
    // Fill mogo
    chassis.pathInterp({
        {24, -47, 1000, {.minSpeed = 127}}, // Ring 1
        {44, -57}, // Ring 2
        {24, -47, 1000, {.forwards = false}}, // Ring 3
        {-24, -47, 1000, {.minSpeed = 127}}, // Ring 4
        {-55, -47, 2000, {.maxSpeed = 50}} // Rings 5 + 6
    });
    pros::delay(900);
    intake.setMode(Intake::modes::INDEX);
    pros::delay(200);
    chassis.swingToHeading(-155, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 127});
    chassis.safeMoveToPoint(-48, -60, 1000, {.maxSpeed = 80}, false); // Grab top wallstake 1
    // Drop goal in bottom left corner
    chassis.turnToPoint(-70, -70, 1000, {.forwards = false});
    chassis.moveTimed(-50, 0, 700, false);
    mogoMech.release(true);
    chassis.moveTimed(50, 0, 500, false);
    // Bottom wallstake
    chassis.safeMoveToPoint(0, -36, 2000);
    chassis.safeMoveToPoint(0, -51, 1000, {}, false);
}