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

        enum class modes { CONTINUOUS, HOLD };
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
        void initialize() {
            std::printf("intake init\n");
            m_hooks->initialize();
            pros::Task task([&](){taskFunct();});
        }

        void waitUntilDone() {
            while (m_hooks->busy()) { pros::delay(10); }
        }

        void forwards(bool force, bool clearQueue = true) {
            m_preroller->forwards();
            if (m_mode == modes::CONTINUOUS) {
                m_hooks->setState(Hooks::states::FORWARDS, force, clearQueue);
            } else {
                m_hooks->setState(Hooks::states::WAIT_FOR_RING, force, clearQueue);
            }
        }

        void reverse(bool force, bool clearQueue = true) {
            m_preroller->reverse();
            m_hooks->setState(Hooks::states::REVERSE, force, clearQueue);
        }

        void idle(bool force) {
            m_preroller->idle();
            m_hooks->setState(Hooks::states::IDLE, force, (m_hooks->busy()) ? false : true);
        }

        void forceIndex() {
            if (m_mode == modes::HOLD) isIndexForced = true;
        }
    protected:
        PrerollerPtr m_preroller = nullptr;
        HooksPtr m_hooks = nullptr;
        ArmPtr m_arm = nullptr;

        modes m_mode = modes::CONTINUOUS;
        states m_state = states::IDLE;

        bool isIndexForced = false;

        void taskFunct() {
            int counter = 0;
            while (true) {
                pros::delay(10);
                bool force = (m_mode == modes::CONTINUOUS) ? true : false;
                bool isArmDown = m_arm->isAtPosition(Arm::idle);
                bool isArmUp = m_arm->isAtPosition(Arm::wall);
                
                switch (m_state) {
                    case states::IDLE:
                        m_preroller->idle();
                        m_hooks->setState(Hooks::states::IDLE, force, force);
                        break;
                    case states::FORWARDS:
                        m_preroller->forwards();
                        if (m_mode == modes::CONTINUOUS) {
                            m_hooks->setState(Hooks::states::FORWARDS, force, force);
                        } else {
                            m_hooks->setState(Hooks::states::WAIT_FOR_RING, force, force);
                        }
                        forwards(false);
                        break;
                    case states::REVERSE:
                        m_preroller->reverse();
                        if (m_mode == modes::CONTINUOUS) {
                            m_hooks->setState(Hooks::states::REVERSE, force, force);
                        } else {
                            m_hooks->setState(Hooks::states::IDLE, force, force);
                        }
                        break;
                }

                m_preroller->update();
                m_hooks->update(m_preroller->hasRing() || isIndexForced, isArmUp);
                isIndexForced = false;
                if (counter % 25 == 0) {
                    // std::cout << *m_hooks.get();
                    counter = 0;
                }
                counter++;
            }
        }
};