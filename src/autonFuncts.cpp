#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
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
    float s = -1;
    // Start position
    chassis.setPose(-54*s, -26.4, -90*s);
    // Grab mogo
    chassis.moveToPoint(-20*s, -26.4, 2000, {.forwards = false, .maxSpeed=55, .earlyExitRange=4});
    robot::printPose();
    pros::delay(200);
    mogoMech.clamp();
    pros::delay(100);
    intake.forwards(true);
    pros::delay(450);
    // Ring 1
    chassis.turnToPoint(-24*s, -42, 1000);
    pros::delay(350);
    chassis.moveToPoint(-24*s, -42, 1000);
    robot::printPose();
    pros::delay(750);
    // Alliance ring
    chassis.turnToPoint(-55*s, 0, 1000);
    pros::delay(300);
    chassis.moveToPoint(-38*s, -27, 3000, {.maxSpeed=60});
    pros::delay(100);
    // chassis.moveToPoint(-54, -7, 3000, {.maxSpeed=30});
    chassis.moveToPoint(-52*s, -7, 3000, {.maxSpeed=30});
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
    chassis.turnToPoint(-23*s, 0, 1000);
    pros::delay(300);
    chassis.moveToPoint(-35*s, 0, 3000, {.maxSpeed=50});
}

void soloAWP() {
    float s = -1;
    // Alliance stake
    chassis.setPose(-59*s, -10, 180*s);
    chassis.moveToPoint(-59*s, 0, 1000, {.forwards = false});
    chassis.turnToHeading(90*s, 1000, {}, false);
    robot::moveTimed(-40, 0, 750);
    pros::delay(100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot::moveTimed(30, 0, 150);
    intake.forwards(true);
    pros::delay(500);
    intake.reverse(true);
    pros::delay(100);
    robot::moveTimed(40, 30*s, 500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Mogo
    chassis.turnToPoint(-24*s, -24, 1000, {.forwards = false});
    pros::delay(250);
    intake.forwards(true);
    // chassis.moveToPoint(-32, -20, 1000, {.forwards = false});
    chassis.moveToPoint(-24*s, -24, 1000, {.forwards = false, .maxSpeed = 50}, false);
    pros::delay(400);
    mogoMech.clamp();
    // Ring 3
    chassis.turnToPoint(-24*s, -48, 1000, {}, false);
    pros::delay(200);
    chassis.moveToPoint(-24*s, -45, 1000, {}, false);
    // Corner
    chassis.turnToPoint(-50*s, -65, 1000, {}, false);
    pros::delay(200);
    chassis.moveToPoint(-50*s, -65, 2000, {.earlyExitRange=10}, true);
    // Arm lift
    intake.setMode(Intake::modes::HOLD);
    arm.moveToPosition(Arm::wall-14.4);
    robot::moveTimed(60, 0, 600);
    pros::delay(250);
    robot::moveTimed(-50, 0, 500);
    arm.moveToPosition(Arm::idle);
    pros::delay(100);
    robot::moveTimed(50, 0, 500);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards(true);
    pros::delay(500);
    chassis.turnToPoint(-30*s, -60, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-30*s, -60, 1000, {}, true);
    pros::delay(200);
    intake.idle(true);
    chassis.waitUntilDone();
    chassis.arcade(0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void twoMogoSolo() {
    float s = 1;
    // Start position
    chassis.setPose(-54*s, -26.4, -90*s);
    // Grab mogo
    chassis.moveToPoint(-20*s, -26.4, 2000, {.forwards = false, .maxSpeed=60});
    robot::printPose();
    mogoMech.clamp();
    pros::delay(100);
    // Preload
    intake.forwards(true);
    pros::delay(200);
    // Ring 2
    chassis.turnToPoint(-24*s, -40, 1000);
    pros::delay(200);
    chassis.moveToPoint(-24*s, -40, 1000);
    robot::printPose();
    pros::delay(300);
    // Cross field
    chassis.turnToPoint(-50*s, 12, 1000, {}, false);
    pros::delay(200);
    chassis.moveToPoint(-50*s, 12, 3000, {.maxSpeed=80}, true);
    // Drop goal
    chassis.waitUntil(24);
    mogoMech.release();
    pros::delay(100);
    // Grab second goal
    chassis.turnToPoint(-24*s, 18, 1000, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(-24*s, 18, 1000, {.forwards=false, .maxSpeed=70}, false);
    mogoMech.clamp();
    pros::delay(200);
    chassis.turnToPoint(-37*s, 40, 1000, {}, false);
    pros::delay(200);
    // Grab last ring
    chassis.moveToPoint(-37*s, 40, 1000, {}, false);
    // Touch bar
    chassis.turnToPoint(chassis.getPose().x, 0, 1000, {}, false);
    robot::moveTimed(50, 0, 800);
    intake.idle(true);
    robot::moveTimed(30, 0, 1000);
}

void elims() {
    float s = 1;
    // Start position
    chassis.setPose(-52*s, 26.4, -90*s);
    // Grab mogo
    chassis.moveToPoint(-20*s, 26.4, 2000, {.forwards = false, .maxSpeed=60, .earlyExitRange=4});
    robot::printPose();
    mogoMech.clamp();
    pros::delay(100);
    intake.forwards(true);
    pros::delay(300);
    // Ring 1
    chassis.turnToPoint(-24*s, 42, 1000);
    pros::delay(200);
    chassis.moveToPoint(-24*s, 42, 1000);
    robot::printPose();
    pros::delay(400);
    // Corner ring
    chassis.turnToPoint(-63*s, 65, 1000, {}, false);
    pros::delay(300);
    intake.setMode(Intake::modes::HOLD);
    arm.moveToPosition(Arm::wall-10);
    chassis.moveToPoint(-63*s, 65, 3000, {.earlyExitRange=10});
    chassis.moveToPoint(-63*s, 63, 3000, {.maxSpeed=50}, false);
    pros::delay(250);
    robot::moveTimed(-50, 0, 300);
    arm.moveToPosition(Arm::idle);
    pros::delay(500);
    robot::moveTimed(50, 0, 300);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards(true);
    pros::delay(500);
    chassis.turnToPoint(-65, -65, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, true);
    pros::delay(200);
    intake.idle(true);
    chassis.moveToPoint(chassis.getPose().x, 0, 1000);
}

} // namespace auton