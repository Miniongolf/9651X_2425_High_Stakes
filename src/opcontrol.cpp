#include "lemlib/util.hpp"
#include "main.h"
#include "pros/motors.h"
#include "robot/helperFuncts.hpp"

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

    // Gamepad init
    Gamepad master(pros::E_CONTROLLER_MASTER);
    Gamepad partner(pros::E_CONTROLLER_PARTNER);

    Button& INTAKE_BUTTON = master.r2;
    Button& OUTTAKE_BUTTON = master.r1;

    Button& MOGO_BUTTON = master.l1;

    Button& ARM_UP_BUTTON = master.x;
    Button& ARM_DOWN_BUTTON = master.a;

    Button& LEFT_DOINKER_BUTTON = master.d_left;
    Button& RIGHT_DOINKER_BUTTON = master.d_right;

    // Subsys init
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.idle(true);
    mogoMech.cancelAutoClamp();
    doinker.retract(Doinker::BOTH);

    printf("-- OPCONTROL STARTING --\n");
    lemlib::Timer matchTimer = 105000;

    bool mogoClampedOnPress;

    while (true) {
        // Update gamepad buttons and sticks
        master.update();
        partner.update();

        if (master.d_up.pressed()) { robot::scoreAllianceStake(pros::E_MOTOR_BRAKE_COAST); }

        /** Timed haptics (45s, 32s, 31s buzz) */
        if (std::fabs(matchTimer.getTimeLeft() - 45000) < 10) {
            master.controller.rumble("-.");
        } else if (std::fabs(matchTimer.getTimeLeft() - 32000) < 10) {
            master.controller.rumble(".");
        } else if (std::fabs(matchTimer.getTimeLeft() - 31000) < 10) {
            master.controller.rumble("...");
        }

        /** Intake */
        if (master.l2) {
            if (OUTTAKE_BUTTON) {
                intake.setMode(Intake::modes::HOLD);
            } else {
                intake.setMode(Intake::modes::INDEX);
            }
        } else {
            intake.setMode(Intake::modes::CONTINUOUS);
        }

        if (INTAKE_BUTTON) {
            intake.forwards(!master.l2, true);
        } else if (OUTTAKE_BUTTON) {
            intake.reverse(true, true);
        } else {
            intake.idle(false);
        }

        // Force index
        if (master.d_down.pressed()) { intake.forceIndex(); }

        // Hooks position trim
        if (partner.d_up) {
            intake.trimHooks(1);
        } else if (partner.d_down) {
            intake.trimHooks(-1);
        } else if (partner.d_left.heldFor(250_msec)) {
            intake.resetHooksOffset();
        }

        /** Arm */
        if (master.l2.released() && master.l2.getLastHoldTime() < 250_msec) {
            double lastTarget = arm.getTargetPosition();
            double target = arm.getTargetPosition() == lemlib::sanitizeAngle(Arm::idle, false) ? Arm::wall : Arm::idle;
            std::printf("Arm toggle %f --> %f\n", arm.getTargetPosition(), lemlib::sanitizeAngle(Arm::idle));
            arm.moveToPosition(target);
        }
        else if (ARM_UP_BUTTON.pressed()) {
            double lastTarget = arm.getTargetPosition();
            double target = arm.getTargetPosition() == lemlib::sanitizeAngle(Arm::idle, false) ? Arm::hang : Arm::idle;
            std::printf("Arm toggle %f --> %f\n", arm.getTargetPosition(), lemlib::sanitizeAngle(Arm::idle));
            arm.moveToPosition(target);
        }

        /** Mogo mech */
        if (MOGO_BUTTON.pressed()) {
            mogoClampedOnPress = mogoMech.isClamped();
            if (mogoMech.isClamped()) {
                mogoMech.release();
            } else {
                mogoMech.requestAutoClamp();
            }
        } else if (MOGO_BUTTON.heldFor(0.25_sec)) {
            mogoMech.requestAutoClamp(false);
        } else if (MOGO_BUTTON.released()) {
            mogoMech.cancelAutoClamp();
            std::cout << "MOGO BUTTON RELEASED " << mogoClampedOnPress << '\n';
            if (!MOGO_BUTTON.lastHeldFor(0.25_sec) && !mogoClampedOnPress) { mogoMech.clamp(); }
        }

        /** Doinker */
        if (LEFT_DOINKER_BUTTON.pressed()) { doinker.toggle(Doinker::LEFT); }
        if (RIGHT_DOINKER_BUTTON.pressed()) { doinker.toggle(Doinker::RIGHT); }

        if (counter % 20 == 0) {
            // std::printf("mogo dist: %f, %d\n", to_mm(mogoMech.getDistance()), mogoMech.isClamped());
        }

        // Chassis
        int throttle = master.stickLeft.y();
        int rightPower = master.stickRight.y();
        int turnPower = master.stickRight.x();

        chassis.arcade(throttle, turnPower * 0.75, false, 0.7);
        // chassis.tank(throttle, rightPower);

        // Telemetry
        counter++;
        pros::delay(10);
    }
}