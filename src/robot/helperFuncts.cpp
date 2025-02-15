#include "robot/helperFuncts.hpp"
#include "constants.hpp" // IWYU pragma: keep
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "subsys/intake/intake.hpp"
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

void safeGrabMogo(float x, float y, int timeout) {
    chassis.safeMoveToPoint(x, y, timeout, {.forwards=false, .earlyExitRange=7}, true);
    chassis.moveToPoint(x, y, timeout, {.forwards=false, .maxSpeed=60}, false);
    mogoMech.clamp(true);
}

void scoreAllianceStake() {
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle();
    chassis.moveTimed(50, 0, 300, false);
    intake.forwards();
    pros::delay(500);
    intake.idle();
}
} // namespace robot