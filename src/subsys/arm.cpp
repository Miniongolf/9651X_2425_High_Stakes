#include "subsys/arm.hpp"

Arm::Arm(std::unique_ptr<pros::MotorGroup> motors, lemlib::PID pid, int rpm)
    : motors(std::move(motors)),
      pid(pid),
      rpm(rpm) {
    this->motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->motors->tare_position();
}

void Arm::moveToAngle(double angle) {
    this->currentState = Arm::state::MOVING;
    double error = lemlib::angleError(this->getAngle(), angle, false, lemlib::AngularDirection::CW_CLOCKWISE);
    int vel = this->pid.update(error);
    this->motors->move_velocity(vel);
}