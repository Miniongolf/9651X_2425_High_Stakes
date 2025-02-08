#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

namespace auton {
void tunePID(bool hasMogo) {
    mogoMech.setState(hasMogo);
    std::printf("Lateral Gains (%f, %f, %f)", chassis.lateralPID.kP, chassis.lateralPID.kI, chassis.lateralPID.kD);
    std::printf("Angular Gains (%f, %f, %f)", chassis.angularPID.kP, chassis.angularPID.kI, chassis.angularPID.kD);
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    // Long turn
    chassis.turnToHeading(135, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    // Short turn
    chassis.turnToHeading(90, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    // Lateral
    chassis.moveToPoint(36, chassis.getPose().y, 5000, {}, false);
    pros::delay(100);
    robot::printPose();
}

void autoTestStuffs() {
    chassis.setPose(0, 0, 0);
    robot::grabMogo(
        {48, 48, 90}, 1000_msec, 10_in, {},
        50, false, 5_in
    );
}

void safeShort() {
    chassis.setPose(-54, -28.5, -90);
    // mogoMech.requestAutoClamp();
    chassis.moveToPoint(-26, -28.5, 2000, {.forwards = false, .minSpeed=40, .earlyExitRange=4});
    chassis.moveToPoint(-20, -22, 2000, {.forwards = false, .maxSpeed = 40});
    robot::printPose();
    mogoMech.clamp();
    intake.forwards(true);
    pros::delay(100);
    chassis.turnToPoint(-24, -42, 1000);
    pros::delay(350);
    chassis.moveToPoint(-24, -42, 1000);
    robot::printPose();
    pros::delay(750);
    chassis.turnToPoint(-54, -7, 1000);
    pros::delay(300);
    chassis.moveToPoint(-54, -7, 3000, {.maxSpeed=30});
    pros::delay(300);
    intake.setMode(Intake::modes::HOLD);
    pros::delay(200);
    arm.moveToPosition(Arm::wall-13);
    robot::printPose();
    pros::delay(250);
    robot::moveTimed(-50, 0, 300);
    pros::delay(150);
    arm.moveToPosition(Arm::idle);
    intake.forwards(true);
    robot::moveTimed(50, 0, 300);
    intake.setMode(Intake::modes::CONTINUOUS);
}

} // namespace auton