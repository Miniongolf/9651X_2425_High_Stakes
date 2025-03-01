#pragma once

#include "robot/subsys/intake/preroller.hpp"
#include "robot/subsys/intake/hooks.hpp"
#include "robot/subsys/arm/arm.hpp"

class Intake {
    public:
        Intake(PrerollerPtr preroller, HooksPtr hooks, ArmPtr arm)
            : m_preroller(std::move(preroller)),
              m_hooks(std::move(hooks)),
              m_arm(std::move(arm)) {}

        enum class modes { CONTINUOUS, INDEX, HOLD };
        enum class states { IDLE, FORWARDS, REVERSE };

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

        void grabTwo() { setMode(modes::INDEX); }

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
        void initialize(Alliance alliance) {
            std::printf("intake init\n");
            m_hooks->initialize(alliance);
            pros::Task task([&]() { taskFunct(); });
        }

        void waitUntilDone() {
            while (m_hooks->busy()) { pros::delay(10); }
        }

        void forwards(bool force = true, bool clearQueue = true) { m_state = states::FORWARDS; }

        void reverse(bool force = true, bool clearQueue = true) { m_state = states::REVERSE; }

        void idle(bool force = true) { m_state = states::IDLE; }

        void forceIndex() {
            if (m_mode == modes::HOLD || m_mode == modes::INDEX) isIndexForced = true;
        }

        void trimHooks(int amount) { m_hooks->poseOffset += amount; }

        void resetHooksOffset() { m_hooks->poseOffset = 0; }

        void setAlliance(Alliance alliance) { m_hooks->m_alliance = alliance; }

        [[nodiscard]] Alliance getAlliance() const { return m_hooks->m_alliance; }
        
        PrerollerPtr m_preroller = nullptr;
        HooksPtr m_hooks = nullptr;
    protected:
        ArmPtr m_arm = nullptr;

        modes m_mode = modes::CONTINUOUS;
        states m_state = states::IDLE;

        bool isIndexForced = false;

        void taskFunct();
};