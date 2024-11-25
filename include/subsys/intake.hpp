#pragma once

#include <cmath>
#include <memory>
#include <queue>
#include "colourRange.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "constants.hpp"

class Intake {
    public:
        enum class state { FORWARDS, REVERSE, STOP };

        /**
         * @brief Construct a new Intake object
         *
         * @param motor unique ptr to a motor object
         */
        Intake(std::unique_ptr<pros::Motor> motor)
            : m_motor(std::move(motor)) {};

        /**
         * @brief Destroy the Intake object
         */
        ~Intake() { m_task.remove(); }

        /**
         * @brief Suspend the intake task
         */
        void suspendTask() { m_task.suspend(); }

        /**
         * @brief Suspend the intake task
         */
        void resumeTask() { m_task.resume(); }

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

        /**
         * @brief Returns whether the optical sensor hue is in a colour range
         * @
         */
        bool isColourInRange(ColourRange colourRange) {
            return false;
        }

        /**
         * @brief Detect if a ring is present
         *
         * @return true if a ring is present
         * @return false if a ring is not present
         */
        bool detectRing() { return false; }

        state m_currState = state::STOP;

        bool filterOn = false;
        std::unique_ptr<pros::Motor> m_motor;
    protected:

        pros::Task m_task = pros::Task {[&]() {
            while (true) {
                pros::delay(10);
                
                switch (m_currState) {
                    case state::FORWARDS: this->move(127); break;
                    case state::REVERSE: this->move(-127); break;
                    case state::STOP: this->move(0); break;
                }
            }
        }};
};