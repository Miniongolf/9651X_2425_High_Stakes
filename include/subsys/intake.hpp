#pragma once

#include <cmath>
#include <queue>
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "colourRange.hpp"

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
         * @note The chain has 40 links, with a 1:1 to a 12t driving sprocket
         */
        double getPose() { return 360-std::fmod(this->motor->get_position() * (12.0/40), 360); }

        int getCurrent() { return this->motor->get_current_draw(); };

        lemlib::PID farPID {0.75, 0, 0};
        lemlib::PID closePID {0.2, 0.01, 1};
    private:
        int jamCurrent = 4000;
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

        explicit Conveyor(Intake& intake, Hooks& hooks, std::unique_ptr<pros::Optical> optical);
        ~Conveyor(){this->task.remove();};

        [[nodiscard]] Conveyor::state getState() const { return this->currState; }

        [[nodiscard]] bool isBusy() const { return this->currState == state::UNJAM || this->currState == state::INDEX; }

        void update();

        void forwards();
        void reverse();
        void stop();
        void unjam();
        void queueIndex();
        void resetIndexQueue() { this->indexQueue = 0; }

        void resumeTask() {this->task.resume();};
        void suspendTask() {this->task.suspend();};

        Intake& intake;
        Hooks& hooks;
        std::unique_ptr<pros::Optical> optical;
    private:
        double getIndexPose();
        void index();
        void idle();

        [[nodiscard]] bool detectRing() const { return this->optical->get_brightness() > 0.03; }
        void moveToIndex(double pose);

        Conveyor::state currState = Conveyor::state::IDLE;
        Conveyor::state prevState = Conveyor::state::IDLE;

        double closeIndexThresh = 7, farIndexThresh = 50;

        int indexQueue = 0;
        double targetIndexPose = 0;

        ColourRange red = ColourRange(0, 25);
        ColourRange blue = ColourRange(150, 250);


        pros::Task task{[&] {
            while (true) {
                pros::delay(10);
                std::printf("Conveyor: %d\n", std::abs(this->hooks.getCurrent()));
                this->update();

                if (this->currState == Conveyor::state::UNJAM) {
                    if (this->prevState == Conveyor::state::REVERSE) { this->hooks.move(50); }
                    else { this->hooks.move(-50); }
                    pros::delay(100);
                    if (!this->hooks.isJammed()) { this->idle(); }
                }

                else if (this->currState == Conveyor::state::INDEX) {
                    this->moveToIndex(0);
                }
            }
        }};
};
