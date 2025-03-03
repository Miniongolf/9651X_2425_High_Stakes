#include "autonFuncts.hpp"

using namespace auton;
void auton::newBlueGoalRush() {
    chassis.setPose(55, -63, -65.5);
    resetSplit();

    doinker.extend();
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    // Goal rush
    chassis.moveTimed(127, 0, 650, false);
    robot::printPose();
    chassis.moveToPoint(55, -70, 1000, {.forwards = false, .minSpeed = 70}, false);
    lemlib::Pose rushedGoalPose = robot::doinkerClampPose();
    nextSplit("Goal rush pull back");

    // Released rushed goal
    chassis.turnToHeading(chassis.getPose().theta + 35, 1000, {.minSpeed = 90, .earlyExitRange = 5});
    chassis.moveTimed(90, 0, 180, false);
    doinker.retract();
    
    // Alliance stake
    chassis.safeMoveToPoint(60, -15, 3000);
    chassis.turnToHeading(0, 1000, {.minSpeed = 0});
    chassis.moveToPoint(60, 9.5, 750, {.maxSpeed = 70});
    chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 1000);
    chassis.moveTimed(-60, 0, 500);
    robot::scoreAllianceStake();
    chassis.moveTimed(50, -20, 500);

    // Grab goal
    robot::safeGrabMogo(24, -24, 1000);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.moveToPoint(-56, 40, 2000);
    doinker.extend();


}