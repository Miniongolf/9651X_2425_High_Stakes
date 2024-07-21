#include "main.h"
#include "autonFuncts.hpp"

/**
 * Robodash auton selector
 */
rd::Selector selector({
    {"Left WP", &auton::leftWP},
    {"Left Max", &auton::leftMax},
    {"Right WP", &auton::rightWP},
    {"Right Rush", &auton::rightRush},
    {"Skills", &auton::skills}
});

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
    int s = isRedAlliance ? 1 : -1;
    robot::resumeTasks();

//    selector.run_auton();
    auton::leftWP();
    // Keep this at the end to suspend tasks. Resume later in opcontrol.
    robot::suspendTasks();
}