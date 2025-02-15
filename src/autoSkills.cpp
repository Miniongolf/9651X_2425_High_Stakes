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
    chassis.setPose(-60, 0, 90);
    intake.forwards();
    pros::delay(300);
    intake.idle();

    // Ring before mogo
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveTimed(127, -50, 300);
    chassis.safeMoveToPoint(-24, 24, 300, 1000); // Ring 1
    // Grab first mogo
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.safeMoveToPoint(-48, 60, 750); // Rings 2 + 3
    chassis.moveTimed(50, 0, 300, false); // Odom reset on top wall
    if (headingCloseTo(0)) { chassis.setPose(chassis.getPose().x, 60, 0); }
    intake.setMode(Intake::modes::INDEX);
    chassis.pathInterp({
        { 0, 58},
        {24, 48},
        { 0, 48, 500, {.forwards=false}}
    }, false);
    arm.moveToPosition(Arm::wall); // Also switches intake to idle continouus
    chassis.turnToHeading(0, 500);
    chassis.moveTimed(70, 0, 250);
    chassis.waitUntilDone();
    intake.reverse(); // Score on wallstake
    pros::delay(500);
    chassis.pathInterp({
        {  0, 58, 500, {.forwards=false}}, // Ring 4
        { 42, 58},
        {  0, 48, 1000, {.forwards=false}},
        {-24, 48, 1000, {.minSpeed=80}}, // Ring 5 (chained to 6)
        {-60, 48} // Ring 6
    });
    chassis.moveTimed(50, 0, 500, false); // Odom reset on left wall
    // Check that angle is close enough to -90
    if (headingCloseTo(-90)) { chassis.setPose(-60, chassis.getPose().y, -90); }

    // Put mogo 1 in corner
    chassis.pathInterp({
        {-48, 48, 500, {.forwards=false}},
        {-60, 60, 750, {.forwards=false}},
    }, false);
    mogoMech.release(true);
    intake.idle();

    // Cross field 
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.safeMoveToPoint(20, -20, 2000);
    chassis.waitUntilDone();
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.safeMoveToPoint(20, -20, 2000, {.maxSpeed=50});
    robot::safeGrabMogo(48, 0, 1000);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards(); // Ring 1+2
    robotAlliance = Alliance::RED;
    chassis.pathInterp({
        {48, 48}, // Ring 3
        {24, 24}, // Ring 4
        {60, 48}, // Ring 5+6 (but blue ring sorted)
    }, false);
    pros::delay(250);
    // Drop mogo 2 in corner
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.safeMoveToPoint(64, 58, 750, {.maxSpeed=80});
    chassis.turnToHeading(-135, 1000);
    pros::delay(250);
    intake.reverse();
    chassis.waitUntilDone();
    mogoMech.release(true);
    
    // Cross back 
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();

    chassis.safeMoveToPoint(-24, -24, 3000);
    robot::safeGrabMogo(-48, -24, 750);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();

    chassis.pathInterp({
        {-24, -48},
        {  0, -60},
        {  0, -48, 500, {.forwards=false}}
    }, false);
    arm.moveToPosition(Arm::wall);
    pros::delay(100);
    chassis.moveTimed(50, 0, 500, false); // Wall stake
    chassis.setPose(0, -60, 180); // Odom reset on bottom wall stake
    intake.reverse();
    pros::delay(500);
    intake.idle();
    chassis.pathInterp({
        {0, -48, 500, {.forwards=false}},
        {45, -48},
        {-48, -48, 2000, {.minSpeed=80, .earlyExitRange=7}},
        {-60, -48, 750, {.maxSpeed=80}}
    }, false);
    chassis.moveTimed(50, 0, 500, false); // Odom reset on left wall
    if (headingCloseTo(-90)) { chassis.setPose(-60, chassis.getPose().y, -90); }
    chassis.pathInterp({
        {-48, -60, 750},
        {-55, -62, 750, {.forwards=false}}
    }, false);
    pros::delay(250);
    mogoMech.release(true);
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.safeMoveToPoint(60, -60, 3000);
    chassis.turnToPoint(60, -48, 400, {}, false);
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.safeMoveToPoint(60, -51, 500, {.maxSpeed=50});
    chassis.swingToHeading(60, lemlib::DriveSide::RIGHT, 500, {.minSpeed=127});
    chassis.pathInterp({
        {35, -18, 750, {.forwards=false}},
        {55, 0, 750, {.forwards=false}},
        {65, 0, 1000, {.forwards=false}}
    }, false);
    robot::scoreAllianceStake();
}