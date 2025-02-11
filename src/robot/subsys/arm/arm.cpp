#include "robot/subsys/arm/arm.hpp" // IWYU pragma: keep

double Arm::idle = -50;
double Arm::wall = 58;

void Arm::taskFunct() {
    int counter = 0;
    while (true) {
        pros::delay(10);
        double error = lemlib::angleError(targetPose, getPosition(), false);
        double voltage = m_pid.update(error) + kF * cos(getPosition(true));
        double maxVolt = 127;
        if (counter % 20 == 0) {
            // std::printf("Arm angle: %f --> %f, | %f\n", getPosition(), targetPose, error);
            // std::printf("isArmUp: %d\n", isAtPosition(wall));
        }
        if (targetPose == idle && isAtPosition(idle, 10)) {
            m_motor->move(0);
            continue;
        }
        voltage = std::clamp(voltage, -maxVolt, maxVolt);
        m_motor->move(voltage);
        counter++;
    }
}

