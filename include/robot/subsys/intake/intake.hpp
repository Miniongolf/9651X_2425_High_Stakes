#pragma once

#include "robot/subsys/intake/preroller.hpp"
#include "robot/subsys/intake/hooks.hpp"

class Intake {
    public:
        Intake(PrerollerPtr preroller, HooksPtr hooks, DistancePtr distance)
            : m_preroller(std::move(preroller)),
              m_hooks(std::move(hooks)) {}

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
        void initialize() {
            m_hooks->initialize();
            pros::Task task([&](){taskFunct();});
            }

        void forwards() {
            m_hooks->setState(Hooks::states::FORWARDS, false, true);
            m_preroller->intake();
        }

        void reverse() {
            m_hooks->setState(Hooks::states::REVERSE, false, true);
            m_preroller->outtake();
        }

        void idle() {
            m_hooks->setState(Hooks::states::IDLE, false, true);
            m_preroller->idle();
        }
    protected:
        PrerollerPtr m_preroller = nullptr;
        HooksPtr m_hooks = nullptr;

        modes m_mode = modes::CONTINUOUS;
        bool isArmUp = false;

        void taskFunct() {
            int counter = 0;
            while (true) {
                pros::delay(10);
                m_preroller->update();
                m_hooks->update(m_preroller->hasRing());
                if (counter % 10 == 0) {
                    std::cout << *m_hooks.get();
                    counter = 0;
                }
            }
        }
};