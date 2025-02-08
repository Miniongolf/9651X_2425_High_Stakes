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
    // Start position
    chassis.setPose(-54, -26.4, -90);
    // Grab mogo
    chassis.moveToPoint(-20, -26.4, 2000, {.forwards = false, .maxSpeed=55, .earlyExitRange=4});
    robot::printPose();
    pros::delay(200);
    mogoMech.clamp();
    pros::delay(100);
    intake.forwards(true);
    pros::delay(450);
    // Ring 1
    chassis.turnToPoint(-24, -42, 1000);
    pros::delay(350);
    chassis.moveToPoint(-24, -42, 1000);
    robot::printPose();
    pros::delay(750);
    // Alliance ring
    chassis.turnToPoint(-55, 0, 1000);
    pros::delay(300);
    chassis.moveToPoint(-38, -27, 3000, {.maxSpeed=60});
    pros::delay(100);
    // chassis.moveToPoint(-54, -7, 3000, {.maxSpeed=30});
    chassis.moveToPoint(-52, -7, 3000, {.maxSpeed=30});
    pros::delay(100);
    intake.setMode(Intake::modes::HOLD);
    arm.moveToPosition(Arm::wall-15);
    robot::printPose();
    pros::delay(250);
    robot::moveTimed(-60, 0, 300);
    pros::delay(150);
    arm.moveToPosition(Arm::idle);
    // intake.forwards(true);
    robot::moveTimed(60, 0, 300);
    intake.setMode(Intake::modes::CONTINUOUS);
    pros::delay(2000);
    chassis.turnToPoint(-23, 0, 1000);
    pros::delay(300);
    chassis.moveToPoint(-35, 0, 3000, {.maxSpeed=50});
}

void soloAWP() {
    chassis.setPose(-59, -7, 180);
    chassis.moveToPoint(-59, 2, 1000, {.forwards = false});
    chassis.turnToPoint(0, 2, 1000);
    chassis.moveToPoint(-65, 2, 1000, {.forwards = false});
    chassis.moveToPoint(-60, 2, 1000, {.forwards = false});
    pros::delay(350);
    intake.forwards(true);
    pros::delay(400);
    chassis.moveToPoint(-57, 2, 1000, {.forwards = false});
    chassis.turnToPoint(-32, -18.5, 1000, {.forwards = false});
    chassis.moveToPoint(-37, -14, 1000, {.forwards = false});
    chassis.moveToPoint(-28, -20, 1000, {.forwards = false, .maxSpeed = 35});
    pros::delay(1600);
    mogoMech.clamp();
}
} // namespace auton