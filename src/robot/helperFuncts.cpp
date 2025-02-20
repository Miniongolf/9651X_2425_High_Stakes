#include "robot/helperFuncts.hpp"
#include "pros/motors.h"

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

void safeGrabMogo(float x, float y, int timeout) {
    chassis.safeMoveToPoint(x, y, timeout, {.forwards=false, .earlyExitRange=7}, true);
    chassis.moveToPoint(x, y, timeout, {.forwards=false, .maxSpeed=60}, false);
    mogoMech.clamp(true);
}

void scoreAllianceStake(pros::motor_brake_mode_e brakeMode) {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle();
    chassis.moveTimed(50, 0, 100, false);
    chassis.brake(brakeMode);
    intake.forwards();
    pros::delay(300);
    intake.idle();
    chassis.setBrakeMode(brakeMode);
}

void hangTier3() {
    std::printf("Hang macro not written yet (rip)\n");
};
} // namespace robot