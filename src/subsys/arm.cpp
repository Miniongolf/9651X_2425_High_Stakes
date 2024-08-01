#include "subsys/arm.hpp"

Arm::Arm(std::unique_ptr<pros::Motor> leftMotor,
         std::unique_ptr<pros::Rotation> leftRot, double leftRatio,
         std::unique_ptr<pros::Motor> rightMotor,
         std::unique_ptr<pros::Rotation> rightRot, double rightRatio,
         lemlib::PID pid, int rpm)
    : leftMotor(std::move(leftMotor)),
      leftRot(std::move(leftRot)),
      leftRatio(leftRatio),
      rightMotor(std::move(rightMotor)),
      rightRot(std::move(rightRot)),
      rightRatio(rightRatio),
      leftPID(pid),
      rightPID(pid),
      rpm(rpm) {
    this->reset();
}

void Arm::reset() {}

void Arm::moveToAngle(double angle) {
    double height = angleToHeight(angle);
//    if (height > 30.25 + 4.5 || height < 8 || this->currState == Arm::state::INACTIVE) return;
    if (angle > 55 || angle < -55 || this->currState == Arm::state::INACTIVE) return;
    this->targetAngle = angle;
}

void Arm::moveToHeight(double height) {
    this->moveToAngle(heightToAngle(height));
}

void Arm::changeAngle(double deltaAngle) {
    this->moveToAngle(this->targetAngle + deltaAngle);
}

void Arm::changeHeight(double deltaHeight) {
    this->moveToHeight(angleToHeight(this->targetAngle) + deltaHeight);
}

void Arm::disconnect() {
    this->currState = Arm::state::INACTIVE;
}

void Arm::connect() {
    this->leftMotor->move(0);
    this->rightMotor->move(0);
    this->currState = Arm::state::HOLD;
}

void Arm::descore() {
    this->currState = Arm::state::DESCORE;
}

double Arm::getLeftAngle() {
    return this->leftRot->get_position() * 0.01 * this->leftRatio;
}

double Arm::getRightAngle() {
    return this->rightRot->get_position() * 0.01 * this->leftRatio;
}