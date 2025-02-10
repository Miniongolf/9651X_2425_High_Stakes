#include "robot/helperFuncts.hpp"
#include "constants.hpp" // IWYU pragma: keep
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <cmath>

namespace robot {
void printPose() {
    chassis.waitUntilDone();
    std::cout << "Chassis pose &t=" << pros::millis() << ": " << format_as(chassis.getPose()) << "\n";
}

void moveTimed(const double throttle, const double steering, const int time) {
    chassis.waitUntilDone();
    chassis.arcade(throttle, steering, true, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

void grabMogo(lemlib::Pose pose, Time timeout, Length leadDist, lemlib::MoveToPoseParams params, double approachSpeed, bool forwards, Length earlyExit) {
    params.forwards = forwards;
    params.minSpeed = std::clamp(approachSpeed, 0.0, 127.0);
    params.earlyExitRange = to_in(earlyExit);
    lemlib::Pose targetPose = {pose.x, pose.y, pose.theta};
    lemlib::Pose interPose = targetPose + lemlib::Pose(
        to_in(mogoOffset) * cos(M_PI_2-lemlib::degToRad(targetPose.theta)),
        to_in(mogoOffset) * sin(M_PI_2-lemlib::degToRad(targetPose.theta)),
        pose.theta
    );
    // Move to inter pose
    chassis.moveToPose(interPose.x, interPose.y, interPose.theta, to_msec(timeout), params, false);
    // Grab mogo
    mogoMech.requestAutoClamp();
    double dist = chassis.getPose().distance(targetPose); // in inches
    chassis.moveToPoint(targetPose.x, targetPose.y, dist * 100, {.forwards=false, .maxSpeed = (float)approachSpeed, .minSpeed = (float)approachSpeed}, false);
    // Exit conditions
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

void swagTurnToPoint(lemlib::Pose point, bool forwards, lemlib::AngularDirection direction, int startSpeed, int endSpeed) {
    int angleMult = (direction == lemlib::AngularDirection::CW_CLOCKWISE) ? 1 : -1;
    startSpeed = std::clamp(std::fabs(startSpeed), 0.0, 127.0);
    endSpeed = std::clamp(std::fabs(endSpeed), 0.0, 127.0);
    double targetHeading = chassis.getPose().angle(point);
    double headingError = lemlib::angleError(targetHeading, chassis.getPose().theta, false, direction);

    // calculate which side to lock for the initial swing
    /* forwards + !cw ==> left
       forwards +  cw ==> right
      !forwards + !cw ==> right
      !forwards +  cw ==> left */
    lemlib::DriveSide lockedSide = (forwards == (direction == lemlib::AngularDirection::CW_CLOCKWISE)) ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT;
    lemlib::DriveSide oppSide = (lockedSide == lemlib::DriveSide::RIGHT) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    // Just swing turn if the angle is small enough
    if (std::fabs(headingError) < 180) {
        chassis.swingToPoint(point.x, point.y, lockedSide, 1000, {.direction = direction, .minSpeed = (float)startSpeed}, false);
        return;
    }
    
    double firstSwingHeading = (forwards) ? chassis.getPose().theta + 90*angleMult : 180 - (chassis.getPose().theta + 90*angleMult);

    chassis.swingToHeading(firstSwingHeading, lockedSide, 1000, {.minSpeed = (float)startSpeed, .earlyExitRange = 30}, false);
    chassis.turnToHeading(targetHeading - 45*angleMult, 750, {.direction = direction, .minSpeed = 127, .earlyExitRange = 10}, false);
    chassis.swingToPoint(point.x, point.y, oppSide, 1000, {.forwards=!forwards, .direction = direction, .minSpeed = (float)endSpeed}, false);
}

void swagTurnToHeading(double theta, bool forwards, lemlib::AngularDirection direction, int startSpeed, int endSpeed) {
    int angleMult = (direction == lemlib::AngularDirection::CW_CLOCKWISE) ? 1 : -1;
    startSpeed = std::clamp(std::fabs(startSpeed), 0.0, 127.0);
    endSpeed = std::clamp(std::fabs(endSpeed), 0.0, 127.0);
    double targetTheta = theta;
    double headingError = lemlib::angleError(theta, chassis.getPose().theta, false, direction);

    // calculate which side to lock for the initial swing
    /* forwards + !cw ==> left
       forwards +  cw ==> right
      !forwards + !cw ==> right
      !forwards +  cw ==> left */
    lemlib::DriveSide lockedSide = (forwards == (direction == lemlib::AngularDirection::CW_CLOCKWISE)) ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT;
    lemlib::DriveSide oppSide = (lockedSide == lemlib::DriveSide::RIGHT) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    // Just swing turn if the angle is small enough
    if (std::fabs(headingError) < 180) {
        chassis.swingToHeading(targetTheta, lockedSide, 1000, {.direction = direction, .minSpeed = (float)startSpeed}, false);
        return;
    }
    
    double firstSwingHeading = (forwards) ? chassis.getPose().theta + 90*angleMult : 180 - (chassis.getPose().theta + 90*angleMult);

    chassis.swingToHeading(firstSwingHeading, lockedSide, 1000, {.minSpeed = (float)startSpeed, .earlyExitRange = 30}, false);
    chassis.turnToHeading(targetTheta - 45*angleMult, 750, {.direction = direction, .minSpeed = 127, .earlyExitRange = 10}, false);
    chassis.swingToHeading((!forwards) ? targetTheta : 180-targetTheta, oppSide, 1000, {.direction = direction, .minSpeed = (float)endSpeed}, false);
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