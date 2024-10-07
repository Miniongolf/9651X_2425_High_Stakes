#include "globals.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    printf("-- OPCONTROL STARTING --\n");

    while (true) {
        // Mogo Mech
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) mogoMech.toggle();

        // Intake + hooks conveyor sys
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.forwards();
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.reverse();
        else intake.stop();

        // Arm
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) baseArm.moveToAngle(90);
        // else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) baseArm.moveToAngle(0);
        master.print(0, 0, "ARM ANGLE: %d", baseArm.getAngle());
        

        // Chassis
        int leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turnPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftPower, turnPower*0.75, false, 1);
        // chassis.tank(leftPower, rightPower);
        pros::delay(10);
    }
}