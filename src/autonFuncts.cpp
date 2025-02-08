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
    chassis.setPose(-59, -10, 180);
    chassis.moveToPoint(-59, 0, 1000, {.forwards = false});
    chassis.turnToHeading(90, 1000, {}, false);
    // chassis.moveToPoint(-69, 0, 1500, {.forwards = false}, false);
    robot::moveTimed(-30, 0, 750);
    pros::delay(100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot::moveTimed(30, 0, 200);
    // chassis.moveToPoint(-61, 2, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    intake.forwards(true);
    pros::delay(500);
    intake.reverse(true);
    pros::delay(100);
    robot::moveTimed(40, 30, 500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToPoint(-24, -24, 1000, {.forwards = false});
    pros::delay(250);
    intake.forwards(true);
    // chassis.moveToPoint(-32, -20, 1000, {.forwards = false});
    chassis.moveToPoint(-24, -24, 1000, {.forwards = false, .maxSpeed = 50}, false);
    pros::delay(400);
    mogoMech.clamp();
    chassis.turnToPoint(-24, -48, 1000, {}, false);
    pros::delay(200);
    chassis.moveToPoint(-24, -48, 1000, {}, false);
    chassis.moveToPoint(-24, -24, 1000, {.forwards = false, .maxSpeed = 50}, false);
    pros::delay(200);
    chassis.turnToPoint(-24, 0, 1000, {}, false);
    pros::delay(200);
    robot::moveTimed(30, 0, 300);
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
} // namespace auton