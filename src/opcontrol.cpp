#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

#define INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_R2
#define OUTTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_R1
#define MOGO_BUTTON pros::E_CONTROLLER_DIGITAL_L1

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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    intake.setMode(Intake::modes::CONTINUOUS);

    pros::Motor armMotor(-12, pros::MotorGears::green);
    armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    printf("-- OPCONTROL STARTING --\n");
    lemlib::Timer wallIntakeTimer = 0;
    lemlib::Timer matchTimer = 105000;

    while (true) {
        // 20s buzz
        if (std::fabs(matchTimer.getTimeLeft() - 20000) < 10) {
            master.rumble("-.");
        } else if (std::fabs(matchTimer.getTimeLeft() - 17000) < 10) {
            master.rumble(".");
        } else if (std::fabs(matchTimer.getTimeLeft() - 16000) < 10) {
            master.rumble("...");
        }

        // Intake
        if (master.get_digital(INTAKE_BUTTON)) {
            intake.forwards();
        } else if (master.get_digital(OUTTAKE_BUTTON)) {
            intake.reverse();
        } else {
            intake.idle();
        }

        // Arm
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            armMotor.move(70);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            armMotor.move(-20);
        } else {
            armMotor.move_velocity(0);
        }

        // Mogo
        if (master.get_digital_new_press(MOGO_BUTTON)) { mogoMech.toggle(); }

        // Chassis
        int leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turnPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftPower, turnPower * 0.85, false, 0.8);

        // chassis.tank(leftPower, rightPower);

        // Telemetry
        if (std::remainder(counter, 10) == 0) {}
        counter++;
        pros::delay(10);
    }
}