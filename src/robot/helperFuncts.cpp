#include "robot/helperFuncts.hpp"

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
    chassis.turnToPoint(x, y, 1000, {.forwards = false, .minSpeed = 0}, false);
    chassis.brake();
    chassis.moveToPoint(x, y, timeout, {.forwards = false, .earlyExitRange = 24}, false);
    chassis.brake();
    chassis.moveToPoint(x, y, 1000, {.forwards = false, .maxSpeed = 40, .minSpeed = 0}, false);
    mogoMech.clamp(true);
    chassis.brake();
}

void scoreAllianceStake() {
    Intake::modes mode = intake.getMode();
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle();
    chassis.moveTimed(45, 0, 95, false);
    chassis.brake();
    pros::delay(20);
    intake.forwards();
    pros::delay(750);
    intake.setMode(mode);
    intake.idle();
}

void scoreWallStake(bool wait, bool push) {
    Intake::modes prevMode = intake.getMode();
    intake.setMode(Intake::modes::CONTINUOUS);
    if (wait) {
        intake.idle();
        pros::delay(700);
    }
    if (push) {
        chassis.arcade(60, 0);
        pros::delay(750);
    }
    intake.reverse(); // Score wallstake
    pros::delay(200);
    chassis.arcade(0, 0);
    pros::delay(550);
    intake.setMode(prevMode);
    intake.idle();
}

void hangTier3() { std::printf("Hang gone (rip ggs)\n"); };
} // namespace robot