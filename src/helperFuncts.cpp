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
    } else {
        ptoPiston.retract();
        activeChassis = &chassis;
        arm.disconnect();
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
    activeChassis->waitUntilDone();
}

void robot::chassisGrabMogo(lemlib::Pose pose, int timeout, lemlib::MoveToPoseParams params) {
    float moveSpeed = 75;

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
    robot::chassisMove(-moveSpeed, 0, 400);
    robot::chassisStop();
    mogoMech.retract();
}