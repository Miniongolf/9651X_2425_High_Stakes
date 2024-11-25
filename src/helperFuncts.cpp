#include "helperFuncts.hpp"
#include "globals.hpp"
#include "lemlib/util.hpp"
#include <cstdio>

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

void clampMogo() {
    mogoMech.extend();
    chassis.lateralPID.setGains(mogoLateralPID);
    chassis.angularPID.setGains(mogoAngularPID);
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

void suspendTasks() {
    arm.suspendTask();
    intake.suspendTask();
}

void resumeTasks() {
    arm.resumeTask();
    intake.resumeTask();
}

void moveRelative(lemlib::Pose relativePose, int timeout, lemlib::MoveToPointParams params, bool async) {
    lemlib::Pose targetPose = relativeToGlobal(relativePose);
    chassis.moveToPoint(targetPose.x, targetPose.y, timeout, params, async);
}

bool grabRing(lemlib::Pose ringPose, int timeout1, int timeout2, double approachDist, lemlib::MoveToPoseParams params,
              bool holdRing) {
    lemlib::Pose interPose(ringPose.x - (intakeOffset + approachDist) * std::cos(lemlib::degToRad(90 - ringPose.theta)),
                           ringPose.y - (intakeOffset + approachDist) * std::sin(lemlib::degToRad(90 - ringPose.theta)),
                           ringPose.theta);

    lemlib::Pose grabPose(ringPose.x - (intakeOffset)*std::cos(lemlib::degToRad(90 - ringPose.theta)),
                          ringPose.y - (intakeOffset)*std::sin(lemlib::degToRad(90 - ringPose.theta)), ringPose.theta);

    chassis.moveToPose(interPose.x, interPose.y, interPose.theta, timeout1, params);
    intake.forwards();
    chassis.moveToPoint(grabPose.x, grabPose.y, timeout2, {.forwards = true, .minSpeed = 70});

    if (holdRing) {
        while (!intake.detectRing() && chassis.isInMotion()) {}
        intake.stop();
    }

    return intake.detectRing();
}

void grabMogo(lemlib::Pose mogoPose, int timeout1, int timeout2, double approachDist,
              lemlib::MoveToPointParams params) {
    lemlib::Pose interPose(mogoPose.x - (mogoOffset - approachDist) * std::cos(lemlib::degToRad(90 - mogoPose.theta)),
                           mogoPose.y - (mogoOffset - approachDist) * std::sin(lemlib::degToRad(90 - mogoPose.theta)),
                           mogoPose.theta);

    std::printf("interPose: (%f, %f, %f)\n", interPose.x, interPose.y, interPose.theta);

    mogoMech.extend();
    chassis.moveToPoint(interPose.x, interPose.y, timeout1, params, false);

    pros::delay(100);
    float angleToMogo = std::atan2(mogoPose.y - chassis.getPose().y, mogoPose.x - chassis.getPose().x);
    lemlib::Pose grabPose(mogoPose.x - (mogoOffset)*std::cos(angleToMogo),
                          mogoPose.y - (mogoOffset)*std::sin(angleToMogo), lemlib::radToDeg(M_PI / 2 - angleToMogo));

    std::printf("grabPose: (%f, %f, %f)\n", grabPose.x, grabPose.y, grabPose.theta);

    chassis.turnToPoint(grabPose.x, grabPose.y, 1000);

    chassis.moveToPoint(grabPose.x, grabPose.y, timeout2, {.forwards = false, .minSpeed = 70}, false);
    mogoMech.retract();
}

void pathInterp(std::vector<PathPoint> path, int waitTime) {
    for (int i = 0; i <= path.size(); i++) {
        PathPoint pathPoint = path[i];
        chassis.turnToPoint(pathPoint.point.x, pathPoint.point.y, pathPoint.turnTimeout, pathPoint.ttpParams,
                            false);
        pros::delay(waitTime);
        robot::printPose();

        pathPoint.mtpParams.forwards = pathPoint.ttpParams.forwards;

        chassis.moveToPoint(pathPoint.point.x, pathPoint.point.x, pathPoint.moveTimeout, pathPoint.mtpParams, false);
        pros::delay(waitTime);
        robot::printPose();

    }
}
} // namespace robot