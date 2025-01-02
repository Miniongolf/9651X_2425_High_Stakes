#include "robot/helperFuncts.hpp"

namespace robot {
void printPose() {
    std::printf("Pose: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

lemlib::Pose relativeToGlobal(lemlib::Pose relativePose) {
    float heading = chassis.getPose(true, true).theta;
    float magnitude = std::sqrt(std::pow(relativePose.x, 2) + std::pow(relativePose.y, 2));

    return {
        chassis.getPose().x + (magnitude * std::cos(heading)),
        chassis.getPose().y + (magnitude * std::sin(heading)),
        chassis.getPose().theta + relativePose.theta
    };
}

void clampMogo(bool wait) {
    if (wait) pros::delay(100);
    mogoMech.extend();
    chassis.lateralPID.setGains(mogoLateralPID);
    chassis.angularPID.setGains(mogoAngularPID);
    if (wait) pros::delay(100);
}

void releaseMogo() {
    mogoMech.retract();
    chassis.lateralPID.setGains(emptyLateralPID);
    chassis.angularPID.setGains(emptyAngularPID);
}

void toggleMogo() {
    (mogoMech.is_extended()) ? releaseMogo() : clampMogo();
}

void moveTimed(const double throttle, const double steering, const int time) {
    chassis.arcade(throttle, steering, true, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

void suspendTasks() {}

void resumeTasks() {}

void moveRelative(lemlib::Pose relativePose, int timeout, lemlib::MoveToPointParams params, bool async) {
    lemlib::Pose targetPose = relativeToGlobal(relativePose);
    chassis.moveToPoint(targetPose.x, targetPose.y, timeout, params, async);
}

void pathInterp(std::vector<PathPoint> path, int waitTime) {
    for (int i = 0; i < path.size(); i++) {
        PathPoint pathPoint = path[i];
        chassis.turnToPoint(pathPoint.point.x, pathPoint.point.y, pathPoint.turnTimeout, pathPoint.ttpParams, false);
        pros::delay(waitTime);
        robot::printPose();

        pathPoint.mtpParams.forwards = pathPoint.ttpParams.forwards;

        chassis.moveToPoint(pathPoint.point.x, pathPoint.point.y, pathPoint.moveTimeout, pathPoint.mtpParams, false);
        pros::delay(waitTime);
        robot::printPose();
    }
}
} // namespace robot