#include "main.h"
#include "autonFuncts.hpp" // IWYU pragma: keep
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp" // IWYU pragma: keep

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/
void autonomous() {
    // Reset stuffs
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    mogoMech.reset();
    intake.idle(true);

    // Put the auton to run here
    // auton::safeShort();
    auton::soloAWP();
    // auton::tunePID(true);

    intake.idle(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}