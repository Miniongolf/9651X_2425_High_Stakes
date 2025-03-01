#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

void auton::sigSAWP() {
    int s = ((robotAlliance == Alliance::RED)) ? 1 : -1;
    chassis.setPose(-50 * s, 24, -90 * s);
    robot::safeGrabMogo(-24 * s, 24, 1000);
    intake.forwards();
    chassis.safeMoveToPoint(-24 * s, 42, 1000, {}, false);
    intake.setMode(Intake::modes::INDEX);
    chassis.safeMoveToPoint(-14 * s, 42, 1000);
    chassis.safeMoveToPoint(-60 * s, 20, 2000);
    chassis.safeMoveToPoint(-60 * s, -11, 1000);
    chassis.swingToHeading(90 * s, lemlib::DriveSide::RIGHT, 1000);
    chassis.moveTimed(-60, 0, 500);
    robot::scoreAllianceStake();
    chassis.moveTimed(50, 0, 500);
    robot::safeGrabMogo(-24 * s, -24, 1000);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.moveToPoint(-24 * s, -22, 1000);
    chassis.moveToPoint(-24*s, -24, 1000, {.forwards = false});
    arm.moveToPosition(Arm::wall);
    chassis.turnToPoint(0, 0, 1000);
    chassis.moveTimed(50, 0, 750);
}