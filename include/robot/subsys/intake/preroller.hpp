#pragma once

#include "util.hpp"

class Preroller {
    public:
        Preroller(MotorPtr prerollerMotor, DistancePtr distance)
            : m_motor(std::move(prerollerMotor)),
              m_distance(std::move(distance)) {
            idle();
        }

        Preroller() = delete;

        enum class states { INTAKE, OUTTAKE, IDLE };

        /**
         * @brief Get the state of the preroller (INTAKE, OUTTAKE, IDLE)
         *
         * @return Preroller::states
         */
        [[nodiscard]] states getState() const { return m_state; }

        /**
         * @brief Detect if a ring is in the preroller
         *
         */
        [[nodiscard]] bool detectRing() const { return m_distance->get() < detectionDistance; }

        void intake() { m_state = states::INTAKE; };

        void outtake() { m_state = states::OUTTAKE; };

        void idle() { m_state = states::IDLE; };

        /**
         * @brief Update the preroller
         *
         */
        void update() {
            switch (m_state) {
                case states::INTAKE: m_motor->move(127); break;
                case states::OUTTAKE: m_motor->move(-127); break;
                case states::IDLE: m_motor->move(0); break;
            }
        }
    protected:
        MotorPtr m_motor = nullptr;
        DistancePtr m_distance = nullptr;

        const int detectionDistance = 100;

        std::atomic<states> m_state = states::IDLE;
};

using PrerollerPtr = std::unique_ptr<Preroller>;