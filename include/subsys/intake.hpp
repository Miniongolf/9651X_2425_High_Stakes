#pragma once

#include <cmath>
#include <memory>
#include <queue>
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "constants.hpp"

class Redirect {
    public:
        enum class state { UP, DOWN };

        /**
         * @brief Construct a new Redirect object
         *
         * @param piston unique ptr to a piston
         * @param optical unique ptr to an optical sensor
         */
        Redirect(std::unique_ptr<pros::adi::Pneumatics> piston, std::unique_ptr<pros::Optical> optical)
            : m_piston(std::move(piston)),
              m_optical(std::move(optical)) {};
    private:
        std::unique_ptr<pros::adi::Pneumatics> m_piston;
        std::unique_ptr<pros::Optical> m_optical;
};

class Intake {
    public:
        enum class state { FORWARDS, REVERSE, STOP };

        /**
         * @brief Construct a new Intake object
         *
         * @param motor unique ptr to a motor object
         */
        explicit Intake(std::unique_ptr<pros::Motor> motor)
            : m_motor(std::move(motor)) {};

        /**
         * @brief Destroy the Intake object
         */
        ~Intake() { m_task.remove(); }

        /**
         * @brief Move the intake forwards
         */
        void forwards() { m_currState = state::FORWARDS; };

        /**
         * @brief Move the intake in reverse
         */
        void reverse() { m_currState = state::REVERSE; };

        /**
         * @brief Stop the intake
         */
        void stop() { m_currState = state::STOP; };

        /**
         * @brief Move the intake
         *
         * @param voltage voltage to move the intake at
         */
        void move(const int voltage) { m_motor->move(voltage); }

        state m_currState = state::STOP;
    protected:
        std::unique_ptr<pros::Motor> m_motor;


        pros::Task m_task = pros::Task {[&]() {
            while (true) {
                pros::delay(10);
                std::printf("INTAKE STATE: %d\n", static_cast<int>(m_currState));
                switch (m_currState) {
                    case state::FORWARDS: this->move(127); break;
                    case state::REVERSE: this->move(-127); break;
                    case state::STOP: this->move(0); break;
                }
            }
        }};
};