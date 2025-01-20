#include "main.h"
#include "gamepad/gamepad.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    Gamepad master(pros::E_CONTROLLER_MASTER);

    // Calibrate chassis    
	chassis.calibrate();
    chassis.setPose(0, 0, 0);

    // Intake init
    intake.initialize();

    // Set pneumatics positions
    mogoMech.initialize();
    mogoMech.release();

    master.controller.clear();

    std::printf("Initialized\n");
    std::printf("Left motor temps: %f, %f, %f\n", leftDrive.get_temperature(0), leftDrive.get_temperature(1), leftDrive.get_temperature(2));
    std::printf("Right motor temps: %f, %f, %f\n", rightDrive.get_temperature(0), rightDrive.get_temperature(1), rightDrive.get_temperature(2));
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}