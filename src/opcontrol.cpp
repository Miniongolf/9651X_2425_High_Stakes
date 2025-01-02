#include "main.h"

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
        
        // Chassis
        int leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turnPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftPower, turnPower*0.85, false, 0.8);

        // chassis.tank(leftPower, rightPower);
        counter++;
        pros::delay(10);
    }
}