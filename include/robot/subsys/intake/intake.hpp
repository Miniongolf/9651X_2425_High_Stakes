#pragma once

#include "robot/subsys/intake/preroller.hpp"
#include "robot/subsys/intake/hooks.hpp"

class Intake {
    public:
        Intake(PrerollerPtr preroller, HooksPtr hooks, DistancePtr distance)
            : m_preroller(std::move(preroller)),
              m_hooks(std::move(hooks)) {}

        enum class states { INTAKE, OUTTAKE, IDLE };
        enum class modes { CONTINUOUS, HOLD };

        /**
         * @brief Get the current mode
         *
         * @return modes
         */
        [[nodiscard]] modes getMode() const { return m_mode; }

        /**
         * @brief Set the mode
         *
         * @param mode
         */
        void setMode(modes mode) { m_mode = mode; }

        /**
         * @brief Get the state of the hooks
         *
         * @return Hooks::states
         */
        [[nodiscard]] Hooks::states getHookState() const { return m_hooks->getState(); }

        /**
         * @brief Get the state of the preroller
         *
         * @return Preroller::states
         */
        [[nodiscard]] Preroller::states getPrerollerState() const { return m_preroller->getState(); }

        /**
         * @brief Initialize the intake
         *
         */
        void initialize() { m_hooks->initialize(); }
    protected:
        PrerollerPtr m_preroller = nullptr;
        HooksPtr m_hooks = nullptr;

        std::atomic<modes> m_mode = modes::CONTINUOUS;
        std::atomic<bool> isArmUp = false;
};