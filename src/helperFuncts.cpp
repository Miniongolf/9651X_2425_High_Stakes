#include "helperFuncts.hpp"

void robot::suspendTasks() {
    arm.suspendTask();
    conveyor.suspendTask();
}

void robot::resumeTasks() {
    arm.resumeTask();
    conveyor.resumeTask();
}

void robot::setPTO(bool state) {
    isPtoActive = state;
    if (state) {
        ptoPiston.extend();
        activeChassis = &ptoChassis;
        arm.connect();
        arm.moveToAngle(arm.getAngle());
        conveyor.canIndex = false;
    } else {
        ptoPiston.retract();
        activeChassis = &chassis;
        arm.disconnect();
        conveyor.canIndex = true;
    }
}

void robot::chassisSetPose(float x, float y, float theta) {
    chassis.setPose(x, y, theta);
    ptoChassis.setPose(x, y, theta);
}

lemlib::Pose robot::chassisGetPose() {
    return chassis.getPose();
}

void robot::printPose(lemlib::Pose pose) {
    std::printf("Pose: %f, %f, %f\n", pose.x, pose.y, pose.theta);
}

void robot::chassisPrintPose() {
    activeChassis->waitUntilDone();
    pros::delay(500);
    std::printf("Chassis ");
    robot::printPose(robot::chassisGetPose());
}

void robot::chassisStop() {
    activeChassis->arcade(0, 0);
}

void robot::chassisMove(int throttle, int turn, int time) {
    activeChassis->arcade(throttle, turn, true);
    pros::delay(time);
    robot::chassisStop();
}

void robot::chassisGrabRing(lemlib::Pose pose, int timeout, lemlib::MoveToPoseParams params) {
    float moveSpeed = 50;
    conveyor.forwards();

    lemlib::Pose newPose(
        pose.x - intakeOffset * std::cos(lemlib::degToRad(90-pose.theta)),
        pose.y - intakeOffset * std::sin(lemlib::degToRad(90-pose.theta)),
        pose.theta
        );

    robot::printPose(newPose);

    params.forwards = true;
//    if (params.minSpeed <= moveSpeed) { params.minSpeed = moveSpeed; }

    activeChassis->moveToPose(newPose.x, newPose.y, newPose.theta, timeout, params);
    robot::chassisPrintPose();
}

void robot::chassisGrabMogo(lemlib::Pose pose, int timeout, lemlib::MoveToPoseParams params) {
    float moveSpeed = 60;

    lemlib::Pose newPose(
        pose.x - mogoOffset * std::cos(lemlib::degToRad(90-pose.theta)),
        pose.y - mogoOffset * std::sin(lemlib::degToRad(90-pose.theta)),
        pose.theta - 180
        );

    robot::printPose(newPose);

    mogoMech.extend();
    params.forwards = false;
//    if (params.minSpeed <= moveSpeed) { params.minSpeed = moveSpeed; }

    activeChassis->moveToPose(newPose.x, newPose.y, newPose.theta, timeout, params);
    activeChassis->waitUntilDone();
    robot::chassisMove(-moveSpeed, 0, 500);
    mogoMech.retract();
    robot::chassisStop();
}

void robot::chassisWallStake(const lemlib::Pose pose, int timeout, bool isAllianceStake, lemlib::MoveToPoseParams params) {
    double offset = isAllianceStake ? allianceStakeOffset : wallStakeOffset;
    double targetAngle = isAllianceStake ? 10 : 60;

    lemlib::Pose newPose(
        pose.x - offset * std::cos(lemlib::degToRad(90-pose.theta)),
        pose.y - offset * std::sin(lemlib::degToRad(90-pose.theta)),
        pose.theta
    );

    robot::printPose(newPose);

    robot::setPTO(true);
    arm.moveToAngle(targetAngle);
    pros::delay(1000);
//    robot::chassisMove(50, -50, 200);
//        if (params.minSpeed <= moveSpeed) { params.minSpeed = moveSpeed; }

    activeChassis->moveToPose(newPose.x, newPose.y, newPose.theta, timeout, params);
    activeChassis->waitUntilDone();
    pros::delay(200);
    arm.moveToAngle(targetAngle - 9);
    pros::delay(1000);
    robot::setPTO(false);
    robot::chassisMove(-100, 0, 300);
}