#include "subsys/arm.hpp"

Arm::Arm(std::unique_ptr<pros::Motor> leftMotor,
         std::unique_ptr<pros::adi::Encoder> leftEnc, double leftRatio,
         std::unique_ptr<pros::Motor> rightMotor,
         std::unique_ptr<pros::adi::Encoder> rightEnc, double rightRatio,
         lemlib::PID pid, int rpm)
    : leftMotor(std::move(leftMotor)),
      leftEnc(std::move(leftEnc)),
      rightMotor(std::move(rightMotor)),
      rightEnc(std::move(rightEnc)),
      leftPID(pid),
      rightPID(pid),
      rpm(rpm) {
    this->reset();
}

void Arm::reset() {
    this->leftMotor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->rightMotor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    this->leftMotor->tare_position();
    this->rightMotor->tare_position();
}

void Arm::moveToAngle(double angle) {
    if (angle < -60 || angle > 80 || this->currState == Arm::state::INACTIVE) return;
    this->targetAngle = angle;
}

void Arm::changeAngle(double deltaAngle) {
    this->moveToAngle(this->targetAngle + deltaAngle);
}

void Arm::disconnect() {
    this->currState = Arm::state::INACTIVE;
}

void Arm::connect() {
    this->leftMotor->move(0);
    this->rightMotor->move(0);
    this->currState = Arm::state::HOLD;
}

double Arm::getLeftAngle() {
    if (this->leftRot == nullptr) return 0;
    return this->leftRot->get_position() * this->leftRatio;
}

double Arm::getRightAngle() {
    if (this->rightRot == nullptr) return 0;
    return this->rightRot->get_position() * this->rightRatio;
}