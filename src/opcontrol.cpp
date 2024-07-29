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
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    // Initialize all subsystems
    robot::resumeTasks();

    if (isRedAlliance) {
        master.print(0, 0, "Red alliance");
    } else {
        master.print(0, 0, "Blue alliance");
    }
    printf("-- OPCONTROL STARTING --\n");

    while (true) {
        // Mogo Mech
        if (master.get_digital_new_press(DIGITAL_L1)) mogoMech.toggle();

        // PTO
        if (master.get_digital_new_press(DIGITAL_L2)) robot::setPTO(!isPtoActive);

        // Intake + hooks conveyor sys
        if (master.get_digital(DIGITAL_R2)) conveyor.forwards();
        else if (master.get_digital(DIGITAL_R1)) conveyor.reverse();
        else if (master.get_digital(DIGITAL_Y)) conveyor.manualInd();
        else if (master.get_digital_new_press(DIGITAL_UP)) conveyor.queueIndex();
        else if (master.get_digital_new_press(DIGITAL_DOWN)) conveyor.resetIndexQueue();
        else conveyor.stop();

        if (master.get_digital(DIGITAL_A)) {
            autonomous();
        }

        // Arm
        if (partner.get_digital(DIGITAL_B)) arm.reset();
        else if (partner.get_digital(DIGITAL_X)) arm.moveToAngle(60);
        else if (partner.get_digital(DIGITAL_A)) arm.moveToAngle(10);
        else if (partner.get_digital(DIGITAL_R1)) arm.changeAngle(0.6);
        else if (partner.get_digital(DIGITAL_R2)) arm.changeAngle(-0.6);
//        else if (partner.get_digital(DIGITAL_DOWN)) arm.hang();
        arm.angleOffset += 0.01 * partner.get_analog(ANALOG_RIGHT_X);

        // Chassis
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        if (master.get_digital(DIGITAL_RIGHT)) {
            activeChassis->arcade(throttle/2, turn/2, false, 0.7);
        } else {
            activeChassis->arcade(throttle, turn, false, 0.7);

        }

        pros::delay(10);
   }
}