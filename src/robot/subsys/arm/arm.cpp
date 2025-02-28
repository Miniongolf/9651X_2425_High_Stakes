#include "robot/subsys/arm/arm.hpp" // IWYU pragma: keep

double Arm::idle = -50;
double Arm::wall = 58;
double Arm::hang = -15;

bool Arm::isJammed() {
    if (m_motor->get_position() == 2147483647) return false;
    if (!jamDetects[jamDetects.size() - 1]) return false;
    int count = 0;
    for (const auto& jam : jamDetects) {
        if (jam) count++;
    };
    return count >= 4;
}

void Arm::taskFunct() {
    int counter = 0;
    while (true) {
        pros::delay(10);

        double error = lemlib::angleError(targetPose, getPosition(), false);
        double voltage = m_pid.update(error) + kF * cos(getPosition(true));
        double maxVolt = 127;
        if (counter % 20 == 0) {
            std::printf("Arm: %d, %f\n", m_motor->get_current_draw(), std::fabs(m_motor->get_actual_velocity()));
            // std::printf("Arm angle: %f --> %f, | %f\n", getPosition(), targetPose, error);
            // std::printf("isArmUp: %d\n", isAtPosition(wall));
        }
        if (targetPose == idle && isAtPosition(idle, 10)) {
            voltage = 0;
        } else {
            voltage = std::clamp(voltage, -maxVolt, maxVolt);
        }
        m_motor->move(voltage);

        // Update jam detection
        bool isJamming = this->isAtPosition(idle, 30) && (this->targetPose != idle) &&
                         m_motor->get_current_draw() >= jamThresh.first &&
                         std::fabs(m_motor->get_actual_velocity()) <= jamThresh.second && (voltage != 0);

        jamDetects.push_back(isJamming);

        if (jamDetects.size() > 5) {
            jamDetects.erase(jamDetects.begin());
        } // Keep the vector size at 5 by popping the oldest element

        if (isJamming) {
        //     std::printf("Arm jammed\n");
        //     jamDetects = {false, false, false, false, false};
            m_motor->move(0);
            pros::delay(100);
        }

        counter++;
    }
}
