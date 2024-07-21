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

/*
void robot::arcadeDrive(int throttle, int turn) {
    if (isPtoActive) {
        ptoChassis.arcade(throttle, turn, true);
    } else {
        chassis.arcade(throttle, turn, true);
    }
}

void robot::chassisTurnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async) {
    if (isPtoActive) {
        ptoChassis.turnToPoint(x, y, timeout, params, async);
    } else {
        chassis.turnToPoint(x, y, timeout, params, async);
    }
}

void robot::chassisTurnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async) {
    if (isPtoActive) {
        ptoChassis.turnToHeading(theta, timeout, params, async);
    } else {
        chassis.turnToHeading(theta, timeout, params, async);
    }
}

void robot::chassisSwingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params, bool async) {
    if (isPtoActive) {
        ptoChassis.swingToPoint(x, y, lockedSide, timeout, params, async);
    } else {
        chassis.swingToPoint(x, y, lockedSide, timeout, params, async);
    }
}

void robot::chassisSwingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params, bool async) {
    if (isPtoActive) {
        ptoChassis.swingToHeading(theta, lockedSide, timeout, params, async);
    } else {
        chassis.swingToHeading(theta, lockedSide, timeout, params, async);
    }
}

void robot::chassisMoveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
    if (isPtoActive) {
        ptoChassis.moveToPose(x, y, theta, timeout, params, async);
    } else {
        chassis.moveToPose(x, y, theta, timeout, params, async);
    }
}
*/