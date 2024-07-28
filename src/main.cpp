#include "main.h"
#include "autonFuncts.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    pros::Controller master(pros::E_CONTROLLER_MASTER);

	chassis.calibrate();
//	ptoChassis.calibrate(false);

    /** @note this must come after chassis init to maintain motor encoder units in deg */
    arm.reset();

    mogoMech.extend();

    robot::setPTO(false);

    lemlib::Timer allianceTimer(5000);

    robot::suspendTasks();

    while (!(master.get_digital(DIGITAL_A) || (master.get_digital(DIGITAL_X)) || allianceTimer.isDone())) {
        if (master.get_digital(DIGITAL_X)) {isRedAlliance = true;}
        else if (master.get_digital(DIGITAL_A)) {isRedAlliance = false;}
    }
    if (isRedAlliance) {
        master.rumble("_");
        master.print(0, 0, "Red alliance");
    } else {
        master.rumble("...");
        master.print(0, 0, "Blue alliance");
    }
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