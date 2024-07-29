#include "main.h"
#include "autonFuncts.hpp"

rd::Selector selector({
    {"Left WP", &auton::leftWP},
    {"Left Max", &auton::leftMax},
    {"Right WP", &auton::rightWP},
    {"Skills", &auton::skills}
});

rd::Console console;

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
    std::printf("isRedAlliance: %d", isRedAlliance);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    robot::resumeTasks();

//    auton::tunePID();
//    selector.run_auton();
    auton::leftMax();


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Keep this at the end to suspend tasks. Resume later in opcontrol.
    robot::suspendTasks();
}