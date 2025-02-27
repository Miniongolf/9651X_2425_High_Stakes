#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

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
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed = 127});
    chassis.moveToPoint(-24, 24, 1000);
    // Grab first mogo (top left corner)
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards(); // Ring 1
    // Fill mogo
    chassis.moveToPose(-24, 30, 65, 1000, {.minSpeed=127});
    chassis.moveToPoint(24, 45, 1250, {.minSpeed = 127, .earlyExitRange = 12}); // Ring 2
    chassis.moveToPoint(42, 53, 1000); // Ring 3
    chassis.moveToPoint(24, 47, 1000, {.forwards = false}); // Ring 4
    chassis.moveToPoint(-60, 47, 3000, {.maxSpeed = 50}, false); // Rings 5+6
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    pros::delay(200);
    chassis.swingToHeading(-25, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 127, .earlyExitRange=10});
    chassis.safeMoveToPoint(-48, 60, 1000, {.maxSpeed = 80}, false); // Grab top wallstake 1
    // Drop goal in corner
    chassis.turnToPoint(-70, 70, 1000, {.forwards = false});
    chassis.moveTimed(-50, 0, 700, false);
    mogoMech.release(true);
    chassis.moveTimed(50, 0, 500, false);
    // Top wallstake
    chassis.safeMoveToPoint(0, 40, 2000);
    chassis.turnToHeading(0, 1000);
    chassis.moveTimed(70, 0, 800, false);
    pros::delay(300);
    chassis.moveTimed(-50, 0, 500);
    arm.moveToPosition(Arm::wall);
    intake.idle();
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.moveTimed(70, 0, 800, false);
    intake.reverse(); // Score wallstake
    pros::delay(750);

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