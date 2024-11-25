#include "globals.hpp"
#include "helperFuncts.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "subsys/arm.hpp"

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

    printf("-- OPCONTROL STARTING --\n");
    robot::resumeTasks();

    while (true) {
        // Mogo Mech
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) mogoMech.toggle();

        // Doinker
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) doinker.toggle();

        // Intake + hooks conveyor sys
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) || partner.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.forwards();
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.reverse();
        else intake.stop();

        // Arm

        /* Driver 2:
        L2 load
        L1 standby
        R1 lady brown extend
        dpad up, dpad down manual
        R2 alliance stake
        Y mogo tip
        A piston extend
        */
        
        // Arm slides
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (arm.m_piston->is_extended()) {
                arm.retract();
                if (arm.targetAngle == armPositions::load) arm.moveToAngle(armPositions::standby);
            } else {
                arm.extend();
                if (arm.targetAngle == armPositions::standby) arm.moveToAngle(armPositions::load);
            }
        }

        // Controller 1 load + standby
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (arm.m_piston->is_extended() || arm.targetAngle == armPositions::standby) {
                arm.moveToAngle(armPositions::load);
            } else { arm.moveToAngle(armPositions::standby); }
        }

        // Manual arm
        if (partner.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) arm.changeAngle(-10);
        else if (partner.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) arm.changeAngle(10);

        // Wall + alliance
        if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            arm.moveToAngle(armPositions::wallStake);
        } else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            arm.moveToAngle(armPositions::allianceStake);
        }

        // Driver 1 arm down
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm.moveToAngle(300);
        }

        // Mogo tip
        if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm.moveToAngle(armPositions::mogoTip);
        }

        // if (counter % 20 == 0) robot::printPose();
        // master.print(0, 0, "ARM ANGLE: %d", arm.getAngle());
        

        // Chassis
        int leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turnPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftPower, turnPower*0.9, false, 0.8);
        // chassis.tank(leftPower, rightPower);
        counter++;
        pros::delay(10);
    }
}