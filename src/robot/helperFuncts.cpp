#include "robot/helperFuncts.hpp"
#include "constants.hpp" // IWYU pragma: keep
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"

namespace robot {
void printPose() {
    std::cout << "Chassis pose &t=" << pros::millis() << ": " << format_as(chassis.getPose()) << "\n";
    std::printf("Pose: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

void moveTimed(const double throttle, const double steering, const int time) {
    chassis.arcade(throttle, steering, true, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

void grabMogo(units::Pose pose, Time timeout, Length leadDist, lemlib::MoveToPoseParams params, double approachSpeed, bool forwards, Length earlyExit) {
    params.forwards = forwards;
    params.minSpeed = std::clamp(approachSpeed, 0.0, 127.0);
    params.earlyExitRange = to_in(earlyExit);
    // Flips over y instead of x
    int s = robotAlliance == Alliance::RED ? 1 : -1;
    units::Pose targetPose = {pose.x, pose.y*s, pose.orientation*s};
    units::Pose interPose = pose - pose.fromPolar(pose.orientation, leadDist + mogoOffset);
    // Move to inter pose
    chassis.moveToPose(to_in(interPose.x), to_in(interPose.y), to_cDeg(interPose.orientation), to_msec(timeout), params, false);
    // Grab mogo
    mogoMech.requestAutoClamp();
    Length dist = units::Vector2D(from_in(chassis.getPose().x) - targetPose.x, from_in(chassis.getPose().y) - targetPose.y).magnitude();
    chassis.moveToPoint(to_in(targetPose.x), to_in(targetPose.y), to_in(dist) * 100, {.forwards=false, .maxSpeed = (float)approachSpeed, .minSpeed = (float)approachSpeed}, false);
    while (chassis.isInMotion()) {
        pros::delay(10);
        // Early exit if autoclamp activates
        if (mogoMech.isClamped()) {
            chassis.cancelMotion();
            break;
        }
    }
    // Clamp anyways even if no mogo is detected
    mogoMech.clamp();
    pros::delay(20);
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