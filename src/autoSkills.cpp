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
    chassis.moveTimed(127, -50, 300);
    chassis.safeMoveToPoint(-24, 24, 300, 1000); // Ring 1
    // Grab first mogo
    robot::safeGrabMogo(-48, 24, 1000);
    // Fill mogo
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.pathInterp({
        {-24, 48},
        {0, 60}
    }, false);
    pros::delay(500);
    chassis.turnToPoint(24, 48, 750);
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    chassis.pathInterp({
        {24, 48},
        {48, 60}
    }, false);
    chassis.swingToPoint(0, 48, lemlib::DriveSide::LEFT, 1000, {.forwards=false});
    
}