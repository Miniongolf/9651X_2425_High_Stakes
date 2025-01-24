#include "main.h"

Gamepad master(pros::E_CONTROLLER_MASTER);
Button& INTAKE_BUTTON = master.r2;
Button& OUTTAKE_BUTTON = master.r1;

Button& MOGO_BUTTON = master.l1;

Button& ARM_UP_BUTTON = master.x;
Button& ARM_DOWN_BUTTON = master.a;

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

    intake.setMode(Intake::modes::CONTINUOUS);
    mogoMech.cancelAutoClamp();

    pros::Motor armMotor(-12, pros::MotorGears::green);
    armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    printf("-- OPCONTROL STARTING --\n");
    lemlib::Timer matchTimer = 105000;

    bool mogoClampedOnPress;

    while (true) {
        // Update gamepad buttons and sticks
        master.update();

        // 20s buzz
        if (std::fabs(matchTimer.getTimeLeft() - 20000) < 10) {
            master.controller.rumble("-.");
        } else if (std::fabs(matchTimer.getTimeLeft() - 17000) < 10) {
            master.controller.rumble(".");
        } else if (std::fabs(matchTimer.getTimeLeft() - 16000) < 10) {
            master.controller.rumble("...");
        }

        // Intake
        intake.setMode((master.l2) ? Intake::modes::HOLD : Intake::modes::CONTINUOUS);
        if (INTAKE_BUTTON) {
            intake.forwards(true, true);
        } else if (OUTTAKE_BUTTON) {
            intake.reverse(true, true);
        } else {
            intake.idle(false);
        }

        // Arm
        if (ARM_UP_BUTTON) {
            armMotor.move(90);
        } else if (ARM_DOWN_BUTTON) {
            armMotor.move(-20);
        } else {
            armMotor.move_velocity(0);
        }

        // Mogo
        // if (MOGO_BUTTON.pressed()) mogoMech.toggle();

        if (MOGO_BUTTON.pressed()) {
            mogoClampedOnPress = mogoMech.isClamped();
            if (mogoMech.isClamped()) { mogoMech.release(); }
            else { mogoMech.requestAutoClamp(); }
        } else if (MOGO_BUTTON.heldFor(0.25_sec)) {
            mogoMech.requestAutoClamp(false);
        } else if (MOGO_BUTTON.released()) {
            mogoMech.cancelAutoClamp();
            std::cout << "MOGO BUTTON RELEASED " << mogoClampedOnPress << '\n';
            if (!MOGO_BUTTON.lastHeldFor(0.25_sec) && !mogoClampedOnPress) {
                mogoMech.clamp();
            }
        }

        if (counter % 10 == 0) {
            // std::cout << "MOGO BUTTON: " << MOGO_BUTTON.getLastHoldTime() << '\n';
            // std::printf("mogo dist: %f, %d\n", to_mm(mogoMech.getDistance()), mogoMech.isClamped());
        }

        // Chassis
        int throttle = master.stickLeft.y();
        int rightPower = master.stickRight.y();
        int turnPower = master.stickRight.x();

        chassis.arcade(throttle, turnPower * 0.7, false, 0.8);
        // chassis.tank(throttle, rightPower);

        // Telemetry
        counter++;
        pros::delay(20);
    }
}