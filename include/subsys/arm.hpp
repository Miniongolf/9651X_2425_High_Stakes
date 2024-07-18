#pragma once

#include <memory>
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"

class Arm {
    public:
        Arm(std::unique_ptr<pros::MotorGroup> motors, lemlib::PID pid, int rpm);

        enum class state {
            MOVING,
            HOLD,
            INACTIVE
        };

        double getAngle() { return this->motors->get_position() * (this->rpm/3600.0); }
        void moveToAngle(double angle);
        void disconnect();
        void connect();
    private:
        std::unique_ptr<pros::MotorGroup> motors;
        lemlib::PID pid;
        int rpm;
        Arm::state currentState = Arm::state::INACTIVE;
};
