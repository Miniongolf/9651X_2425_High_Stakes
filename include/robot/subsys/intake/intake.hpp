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
            std::printf("intake init\n");
            m_hooks->initialize();
            pros::Task task([&](){taskFunct();});
        }

        void waitUntilDone() {
            while (m_hooks->busy()) { pros::delay(10); }
        }

        void forwards(bool force, bool clearQueue = true) {
            m_preroller->intake();
            if (m_mode == modes::CONTINUOUS) {
                m_hooks->setState(Hooks::states::FORWARDS, force, clearQueue);
            } else {
                m_hooks->setState(Hooks::states::WAIT_FOR_RING, force, clearQueue);
            }
        }

        void reverse(bool force, bool clearQueue = true) {
            m_preroller->outtake();
            m_hooks->setState(Hooks::states::REVERSE, force, clearQueue);
        }

        void idle(bool force) {
            m_preroller->idle();
            m_hooks->setState(Hooks::states::IDLE, force, (m_hooks->busy()) ? false : true);
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
                if (counter % 25 == 0) {
                    // std::cout << *m_hooks.get();
                    counter = 0;
                }
                counter++;
            }
        }
};