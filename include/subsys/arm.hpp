#pragma once

#include <memory>
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"

class Arm {
    public:
        Arm(std::unique_ptr<pros::Motor> leftMotor, std::unique_ptr<pros::Motor> rightMotor, lemlib::PID pid, int rpm);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD,
            INACTIVE
        };

        void reset();

        double getAngle() { return (this->getLeftAngle() + this->getRightAngle())/2; }
        void moveToAngle(double angle);
        void changeAngle(double deltaAngle);
        void disconnect();
        void connect();

        void resumeTask() {this->task.resume();};
        void suspendTask() {this->task.suspend();};

    private:
        double getLeftAngle() { return this->leftMotor->get_position() * (this->rpm/360.0); }
        double getRightAngle() { return this->rightMotor->get_position() * (this->rpm/360.0); }

        std::unique_ptr<pros::Motor> leftMotor, rightMotor;
        lemlib::PID leftPID, rightPID;
        int rpm;
        double targetAngle = 0;
        Arm::state currState = Arm::state::INACTIVE;

        pros::Task task = pros::Task {[&] {
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
        }};;
};
