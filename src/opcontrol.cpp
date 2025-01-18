#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/motors.hpp"

#define INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_R2
#define OUTTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_R1

#define MOGO_BUTTON pros::E_CONTROLLER_DIGITAL_L1

#define ARM_UP_BUTTON pros::E_CONTROLLER_DIGITAL_X
#define ARM_DOWN_BUTTON pros::E_CONTROLLER_DIGITAL_A

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    int counter = 0;
    Gamepad master(pros::E_CONTROLLER_MASTER);

    intake.setMode(Intake::modes::CONTINUOUS);

    pros::Motor armMotor(-12, pros::MotorGears::green);
    armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    printf("-- OPCONTROL STARTING --\n");
    lemlib::Timer matchTimer = 105000;

    while (true) {
        // Update gamepad buttons and sticks
        master.update();

        // 20s buzz
        if (std::fabs(matchTimer.getTimeLeft() - 20000) < 10) {
            master.controller.rumble("-.");
        } else if (std::fabs(matchTimer.getTimeLeft() - 17000) < 10) {
            master.controller.rumble(".");
        } else if (std::fabs(matchTimer.getTimeLeft() - 16000) < 10) {
            master.controller.rumble("...");
        }

        // Intake
        if (master[INTAKE_BUTTON]) {
            intake.forwards();
        } else if (master[OUTTAKE_BUTTON]) {
            intake.reverse();
        } else {
            intake.idle();
        }

        // Arm
        if (master[ARM_UP_BUTTON]) {
            armMotor.move(70);
        } else if (master[ARM_DOWN_BUTTON]) {
            armMotor.move(-20);
        } else {
            armMotor.move_velocity(0);
        }

        // Mogo
        if (master[MOGO_BUTTON]->heldFor(1000_msec)) {
            mogoMech.requestAutoClamp();
        } else if (master[MOGO_BUTTON]->released()) {
            mogoMech.toggle();
        }
        if (master[MOGO_BUTTON]->released()) { mogoMech.cancelAutoClamp(); }

        // Chassis
        int leftPower = master.stickLeft.y();
        int rightPower = master.stickRight.y();
        int turnPower = master.stickRight.x();

        chassis.arcade(leftPower, turnPower * 0.85, false, 0.8);
        // chassis.tank(leftPower, rightPower);

        // Telemetry
        counter++;
        pros::delay(25);
    }
}