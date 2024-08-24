#pragma once

#include <cmath>
#include <queue>
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "constants.hpp"

class Intake {
    public:
        enum class state {
            FORWARDS,
            REVERSE,
            STOP,
            IDLE,
            UNJAM
        };

        explicit Intake(pros::Motor motor, pros::Optical optical);

        [[nodiscard]] bool isJammed() const { return this->motor.get_current_draw() > this->jamCurrent; }

        void move(const int voltage) const { this->motor.move(voltage); }

        void forwards();
        void reverse();
        void stop();

    private:
        const int jamCurrent = 3200;
        void idle();
        void unjam();
    
        pros::Motor motor;
        pros::Optical optical;
};