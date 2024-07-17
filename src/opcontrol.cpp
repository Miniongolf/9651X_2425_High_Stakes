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
   pros::Controller master(pros::E_CONTROLLER_MASTER);

//   int intakeDir;

   while (true) {
//       intakeDir = master.get_digital(DIGITAL_R1) ? -1 : master.get_digital(DIGITAL_R2) ? 1 : 0;
//
//       intake.move(intakeDir*127);
//
//       if (master.get_digital(DIGITAL_A)) hooks.move(40);
//       else if (master.get_digital(DIGITAL_L2)) hooks.move(-intakeDir*50);
//       else hooks.move(intakeDir*127);

       conveyor.update();
       if (master.get_digital(DIGITAL_R2)) conveyor.forwards();
       else if (master.get_digital(DIGITAL_R1)) conveyor.reverse();
       else if (master.get_digital(DIGITAL_UP)) conveyor.queueIndex();
       else conveyor.idle();

       if (master.get_digital_new_press(DIGITAL_L1)) mogoMech.toggle();

       // Arcade control scheme
       int throttle = master.get_analog(ANALOG_LEFT_Y);
       int turn = master.get_analog(ANALOG_RIGHT_X);

       chassis.arcade(throttle, turn, true); // Moves the robot based on the joystick values
       std::cout << chassis.getPose().x << ", " << chassis.getPose().y << " | " << chassis.getPose().theta << "\n";
       pros::delay(10); // Run for 10 ms then update
   }
}