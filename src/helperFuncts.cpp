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

void robot::chassisGrabRing(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params) {
    conveyor.forwards();
    lemlib::Pose newPose(
        x - intakeOffset * std::cos(lemlib::degToRad(90-theta)),
        y - intakeOffset * std::sin(lemlib::degToRad(90-theta)),
        theta
    );
    activeChassis->moveToPose(newPose.x, newPose.y, theta, timeout, params);
    activeChassis->waitUntilDone();
}

void robot::chassisGrabMogo(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params) {
    lemlib::Pose newPose(
        x - mogoOffset * std::cos(lemlib::degToRad(90-theta)),
        y - mogoOffset * std::sin(lemlib::degToRad(90-theta)),
        theta
    );

    mogoMech.extend();
    lemlib::MoveToPoseParams newParams = params;
    newParams.forwards = false;
    activeChassis->moveToPose(newPose.x, newPose.y, theta, timeout, newParams);
    activeChassis->waitUntilDone();
    mogoMech.retract();
}