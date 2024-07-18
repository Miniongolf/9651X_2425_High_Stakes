#include "subsys/arm.hpp"

Arm::Arm(std::unique_ptr<pros::Motor> leftMotor, std::unique_ptr<pros::Motor> rightMotor, lemlib::PID pid, int rpm)
    : leftMotor(std::move(leftMotor)),
      rightMotor(std::move(rightMotor)),
      leftPID(pid),
      rightPID(pid),
      rpm(rpm) {
    this->reset();
    pros::Task armTask{[&] {
        while (true) {
            double error = lemlib::angleError(this->getAngle(), this->targetAngle, false);
            double leftError = lemlib::angleError(this->getLeftAngle(), this->targetAngle, false);
            double rightError = lemlib::angleError(this->getRightAngle(), this->targetAngle, false);
            std::printf("Arm: %f, %f\n", this->getLeftAngle(), this->getRightAngle());
            if (this->currState == Arm::state::INACTIVE) continue;

            if (std::fabs(error) <= 5) { this->currState = Arm::state::HOLD; }
            else { this->currState = Arm::state::MOVING; }

            if (this->currState == Arm::state::MOVING) {
                double leftVel = this->leftPID.update(leftError),
                       rightVel = this->rightPID.update(rightError);
                this->leftMotor->move(leftVel);
                this->rightMotor->move(rightVel);
            } else if (this->currState == Arm::state::HOLD) {
                this->leftMotor->move(0);
                this->rightMotor->move(0);
            }
            pros::delay(10);
        }
    }};
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