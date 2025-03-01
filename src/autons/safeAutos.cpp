#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/helperFuncts.hpp"

void auton::twoRing(bool positive) {
    int s = ((robotAlliance == Alliance::RED) && positive) ? 1 : -1;
    chassis.setPose(-53 * s, -17, -90 * s);
    chassis.moveToPoint(-31 * s, -17, 1000);
    chassis.swingToHeading(0, lemlib::DriveSide::LEFT * s, 750);
    chassis.waitUntil(50);
    mogoMech.clamp();
    chassis.turnToPoint(-24 * s, -48, 600, {}, false);
    intake.forwards();
    chassis.moveToPoint(-24 * s, -42, 750);
    chassis.swingToHeading(90 * s, lemlib::DriveSide::RIGHT * s, 750);
    chassis.safeMoveToPoint(-24 * s, -24, 1000, {}, false);
    chassis.turnToPoint(0, 0, 600);
    arm.moveToPosition(Arm::wall);
    chassis.moveTimed(50, 0, 750);
}

void auton::safeAWP(bool positive) {
    int s = ((robotAlliance == Alliance::RED) && positive) ? 1 : -1;
    chassis.setPose(-55 * s, -16, 0);
    chassis.moveTimed(50, 0, 600);
    chassis.swingToHeading(90 * s, lemlib::DriveSide::LEFT * s, 600);
    chassis.moveTimed(-70, 0, 500);
    robot::scoreAllianceStake();
    chassis.moveTimed(50, 20 * s, 300);
    robot::safeGrabMogo(-24 * s, -24, 1250);
    intake.forwards();
    chassis.safeMoveToPoint(-24 * s, -45, 1000);
    chassis.safeMoveToPoint(-43 * s, -61, 1000);
    chassis.moveTimed(80, 10 * s, 1000);
    chassis.moveTimed(127, 0, 750);
    chassis.moveToPoint(-24 * s, -48, 1000, {.forwards = false}, false);
    arm.moveToPosition(Arm::wall);
    chassis.moveToPose(-13 * s, -13, 45 * s, 1000, {.maxSpeed = 50});
}
