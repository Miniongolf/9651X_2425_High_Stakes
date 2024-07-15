#pragma once

#include "pros/motor_group.hpp"
#include <queue>

class Intake {
    public:
        explicit Intake(std::unique_ptr<pros::Motor> intakeMotor);

        [[nodiscard]] bool hasRing() const { return motor->get_torque() > this->threshTorque; }

        void move(int speed) { this->motor->move(speed); };

    private:
        std::unique_ptr<pros::Motor> motor;
        const float threshTorque = 0;
};

class Hooks {
    public:
        explicit Hooks(std::unique_ptr<pros::Motor> hooksMotor);

        [[nodiscard]] bool isJammed() const { return motor->get_torque() > this->jamTorque; }

        void move(int speed) { this->motor->move(speed); };

        double getPose() { return this->motor->get_position(); }

    private:
        double jamTorque = 0.1;
        std::unique_ptr<pros::Motor> motor;
};


class Conveyor {
    public:
        enum class state {
            FORWARDS,
            REVERSE,
            UNJAM,
            INDEX,
            IDLE
        };

        explicit Conveyor(Intake& intake, Hooks& hooks);

        [[nodiscard]] Conveyor::state getState() const { return this->currState; }

        void update();

        void forwards();
        void reverse();
        void idle();
        void unjam();
        void queueIndex();

    private:
        double getIndexPose();

        Intake& intake;
        Hooks& hooks;
        Conveyor::state currState = Conveyor::state::IDLE;
        Conveyor::state prevState = Conveyor::state::IDLE;

        std::queue<double> indexQueue;
};
