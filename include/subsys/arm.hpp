#pragma once

#include <memory>
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"

class Arm {
    public:
        Arm(std::unique_ptr<pros::Motor> leftMotor, std::unique_ptr<pros::Motor> rightMotor, lemlib::PID pid, int rpm);

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
    private:
        double getLeftAngle() { return this->leftMotor->get_position() * (this->rpm/360.0); }
        double getRightAngle() { return this->rightMotor->get_position() * (this->rpm/360.0); }

        std::unique_ptr<pros::Motor> leftMotor, rightMotor;
        lemlib::PID leftPID, rightPID;
        int rpm;
        double targetAngle = 0;
        Arm::state currState = Arm::state::INACTIVE;
};
