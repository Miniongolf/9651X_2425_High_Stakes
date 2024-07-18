#pragma once

#include <cmath>
#include <queue>
#include "pros/motor_group.hpp"
#include "lemlib/util.hpp"

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

        /**
         * @return Whether the hooks are jammed
         */
        [[nodiscard]] bool isJammed() const { return std::abs(motor->get_current_draw()) > this->jamCurrent; }

        /**
         * Moves the hooks at a given voltage
         * @param speed
         */
        void move(int speed) { this->motor->move(speed); };

        /**
         * @return The current pose of the hooks in degrees (with a full revolution of a chain being 360º)
         * @note The chain has 44 links, with a 1:1 to a 12t driving sprocket
         */
        double getPose() { return std::fmod(this->motor->get_position() * (12.0/44), 360); }

        int getCurrent() { return this->motor->get_current_draw(); };

        lemlib::PID farPID {1, 0, 0};
        lemlib::PID closePID {0.3, 0.01, 0.5};
    private:

        int jamCurrent = 2000;
        std::unique_ptr<pros::Motor> motor;
};


class Conveyor {
    public:
        enum class state {
            FORWARDS,
            REVERSE,
            STOP,
            UNJAM,
            INDEX,
            IDLE
        };

        explicit Conveyor(Intake& intake, Hooks& hooks);

        [[nodiscard]] Conveyor::state getState() const { return this->currState; }

        [[nodiscard]] bool getIsBusy() const { return this->isBusy; }

        void update();

        void forwards();
        void reverse();
        void stop();
        void unjam();
        void queueIndex();

        Intake& intake;
        Hooks& hooks;
    private:
        double getIndexPose();
        void index();

        void idle();

        bool isBusy = false;

        double closeIndexThresh = 3, farIndexThresh = 50;

        Conveyor::state currState = Conveyor::state::IDLE;
        Conveyor::state prevState = Conveyor::state::IDLE;

        std::queue<double> indexQueue;
};
