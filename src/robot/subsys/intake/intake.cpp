#include "robot/subsys/intake/intake.hpp" // IWYU pragma: keep

void Intake::taskFunct() {
    int counter = 0;
    while (true) {
        pros::delay(10);
        bool force = (m_mode == modes::INDEX) ? false : true;
        bool isArmDown = m_arm->isAtPosition(Arm::idle);
        bool isArmUp = m_arm->isAtPosition(Arm::wall);

        if (isArmUp) { setMode(modes::CONTINUOUS); }

        switch (m_state) {
            case states::IDLE:
                m_preroller->idle();
                m_hooks->setState(Hooks::states::IDLE, true, true);
                break;
            case states::FORWARDS:
                m_preroller->forwards();
                if (m_mode == modes::CONTINUOUS) {
                    m_hooks->setState(Hooks::states::FORWARDS, true, true);
                } else if (m_mode == modes::INDEX) {
                    m_hooks->setState(Hooks::states::INDEX, true, true);
                } else {
                    m_hooks->setState(Hooks::states::IDLE, true, true);
                }
                break;
            case states::REVERSE:
                m_preroller->reverse();
                if (m_mode == modes::CONTINUOUS) {
                    m_hooks->setState(Hooks::states::REVERSE, true, true);
                } else {
                    m_hooks->setState(Hooks::states::IDLE, true, true);
                }
                break;
        }

        m_preroller->update();
        m_hooks->update(m_preroller->hasRing(), isIndexForced, isArmUp, m_arm->isJammed());
        isIndexForced = false;
        if (counter % 25 == 0) {
            // std::cout << *m_hooks.get();
        }
        counter++;
    }
};