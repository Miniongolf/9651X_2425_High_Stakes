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

    printf("-- OPCONTROL STARTING --\n");
    while (true) {
        // Update all subsys


        // Mogo Mech
        if (master.get_digital_new_press(DIGITAL_L1)) mogoMech.toggle();

        // PTO
        if (master.get_digital_new_press(DIGITAL_L2)) robot::setPTO(!isPtoActive);

        // Intake + hooks conveyor sys
        if (master.get_digital(DIGITAL_R2)) conveyor.forwards();
        else if (master.get_digital(DIGITAL_R1)) conveyor.reverse();
        else if (master.get_digital(DIGITAL_UP)) conveyor.queueIndex();
        else conveyor.stop();

        if (master.get_digital(DIGITAL_A)) {
//            std::cout << chassis.getPose().x << ", " << chassis.getPose().y << " | " << chassis.getPose().theta << "\n";
//            console.printf("Chassis pose: %f, %f | %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
//            std::printf("Arm angle: %f, %f | %f\n", arm.getAngle(), arm.getLeftAngle(), arm.getRightAngle());
        }

        // std::printf("%f | %d | %d\n", conveyor.hooks.getPose(), conveyor.hooks.getCurrent(), static_cast<int>(conveyor.getState()));
        // std::cout << conveyor.hooks.getPose() << ' ' << static_cast<int>(conveyor.getState()) << ' ' << "\n";

        // Arm
        if (partner.get_digital(DIGITAL_B)) arm.reset();
        if (partner.get_digital(DIGITAL_X)) arm.moveToAngle(60);
        if (partner.get_digital(DIGITAL_A)) arm.moveToAngle(0);
        if (partner.get_digital(DIGITAL_R1)) arm.changeAngle(0.6);
        if (partner.get_digital(DIGITAL_R2)) arm.changeAngle(-0.6);
        arm.angleOffset += 0.05 * partner.get_analog(ANALOG_RIGHT_X);

        // Chassis
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        activeChassis->arcade(throttle, turn, false, 0.7);

        pros::delay(10);
   }
}