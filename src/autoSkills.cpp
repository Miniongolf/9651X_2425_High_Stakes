#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

bool headingCloseTo(double target) { return std::fabs(lemlib::angleError(target, chassis.getPose().theta)) <= 5; };

// void auton::skills() {
//     // Disable colour sort
//     robotAlliance = Alliance::NONE;

//     // Start pose (in front of alliance stake)
//     chassis.setPose(-64, 0, 90);
//     intake.forwards();
//     pros::delay(500);
//     intake.idle();

//     // Ring before mogo
//     intake.setMode(Intake::modes::INDEX);
//     intake.forwards();
//     chassis.moveTimed(50, 0, 500);
//     chassis.swingToPoint(-24, 24, lemlib::DriveSide::LEFT, 750, {.minSpeed = 127});
//     chassis.moveToPoint(-24, 24, 1000);
//     // Grab first mogo (top left corner)
//     robot::safeGrabMogo(-48, 24, 1000);
//     // Fill mogo
//     intake.setMode(Intake::modes::CONTINUOUS);
//     intake.forwards(); // Ring 1
//     chassis.moveToPose(-24, 30, 65, 1000);
//     chassis.moveToPoint(24, 48, 1250, {.minSpeed = 100, .earlyExitRange = 12}, false); // Ring 2
//     chassis.moveToPoint(44, 57, 1000, {.maxSpeed = 50}); // Ring 3
//     chassis.safeMoveToPoint(24, 48, 1000, {.forwards = false});
//     // Rings 4, 5, 6
//     chassis.safeMoveToPoint(-60, 48, 500, 3000, {.earlyExitRange = 0}, {.maxSpeed = 50}, false);
//     pros::delay(500);
//     intake.setMode(Intake::modes::INDEX);
//     chassis.swingToHeading(-25, lemlib::DriveSide::LEFT, 1000, {.minSpeed=70});
//     chassis.safeMoveToPoint(-48, 60, 1000, {.maxSpeed=80}, false); // Grab top wallstake 1
//     // Drop goal in corner
//     chassis.turnToPoint(-70, 70, 1000, {.forwards = false});
//     chassis.moveTimed(-50, 0, 700, false);
//     mogoMech.release(true);
//     chassis.moveTimed(50, 0, 500);
//     // Top wallstake
//     chassis.safeMoveToPoint(10, 60, 1500, {.earlyExitRange = 18});
//     chassis.safeMoveToPoint(10, 60, 1500, {.maxSpeed = 60}); // Grab top wallstake 2
//     chassis.swingToPoint(0, 70, lemlib::DriveSide::RIGHT, 1000);
//     pros::delay(300);
//     arm.moveToPosition(Arm::wall);
//     intake.idle();
//     intake.setMode(Intake::modes::CONTINUOUS);
//     chassis.moveTimed(70, 0, 800, false);
//     intake.reverse(); // Score
//     pros::delay(1000);

//     // Empty mogo (top right corner)
//     intake.idle();
//     chassis.swingToPoint(48, 48, lemlib::DriveSide::RIGHT, 1000, {.forwards = false, .minSpeed=80}, false);
//     arm.moveToPosition(Arm::idle);
//     chassis.safeMoveToPoint(48, 48, 1000, {.maxSpeed = 80});
//     robot::safeGrabMogo(60, 24, 1000);
//     intake.setMode(Intake::modes::HOLD);
//     intake.forwards();
//     chassis.safeMoveToPoint(60, 52, 1000, {.maxSpeed = 70});
//     chassis.swingToPoint(70, 70, lemlib::DriveSide::LEFT, 600, {.forwards = false}, false);
//     chassis.moveTimed(50, 0, 500);
//     mogoMech.release(true);
//     chassis.moveTimed(-50, 0, 750);

//     chassis.safeMoveToPoint(36, 36, 1000);
// }

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
    chassis.moveToPose(-24, 30, 65, 1000);
    chassis.moveToPoint(24, 48, 1250, {.minSpeed = 100, .earlyExitRange = 12}, false); // Ring 2
    chassis.moveToPoint(44, 57, 1000, {.maxSpeed = 50}); // Ring 3
    chassis.safeMoveToPoint(24, 48, 1000, {.forwards = false});
    // Rings 4, 5, 6
    chassis.safeMoveToPoint(-60, 48, 500, 3000, {.earlyExitRange = 0}, {.maxSpeed = 50}, false);
    pros::delay(500);
    intake.setMode(Intake::modes::INDEX);
    chassis.swingToHeading(-25, lemlib::DriveSide::LEFT, 1000, {.minSpeed=70});
    chassis.safeMoveToPoint(-48, 60, 1000, {.maxSpeed=80}, false); // Grab top wallstake 1
    // Drop goal in corner
    chassis.turnToPoint(-70, 70, 1000, {.forwards = false});
    chassis.moveTimed(-50, 0, 700, false);
    mogoMech.release(true);
    chassis.moveTimed(50, 0, 500);
    // Top wallstake
    chassis.safeMoveToPoint(10, 60, 1500, {.earlyExitRange = 18});
    chassis.safeMoveToPoint(10, 60, 1500, {.maxSpeed = 60}); // Grab top wallstake 2
    chassis.swingToPoint(0, 70, lemlib::DriveSide::RIGHT, 1000);
    pros::delay(300);
    arm.moveToPosition(Arm::wall);
    intake.idle();
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.moveTimed(70, 0, 800, false);
    intake.reverse(); // Score
    pros::delay(1000);

    // Empty mogo (top right corner)
    intake.idle();
    chassis.swingToPoint(48, 48, lemlib::DriveSide::RIGHT, 1000, {.forwards = false, .minSpeed=80}, false);
    arm.moveToPosition(Arm::idle);
    chassis.safeMoveToPoint(48, 48, 1000, {.maxSpeed = 80});
    robot::safeGrabMogo(60, 24, 1000);
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.safeMoveToPoint(60, 52, 1000, {.maxSpeed = 70});
    chassis.swingToPoint(70, 70, lemlib::DriveSide::LEFT, 600, {.forwards = false}, false);
    chassis.moveTimed(50, 0, 500);
    mogoMech.release(true);
    chassis.moveTimed(-50, 0, 750);

    chassis.safeMoveToPoint(36, 36, 1000);
}