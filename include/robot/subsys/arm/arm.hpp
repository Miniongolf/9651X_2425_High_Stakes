#pragma once

#include "util.hpp"

class Arm {
    public:
        Arm(MotorPtr armMotor, lemlib::PID pid, double kF, double gearRatio)
            : m_motor(std::move(armMotor)),
              m_pid(pid),
              kF(kF),
              m_ratio(gearRatio) {
            m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        }

        constexpr static double idle = -50;
        constexpr static double wall = 47;
        void initialize() {
            m_motor->tare_position();
            // setPosition(idle);
            pros::Task task([&](){taskFunct();});
        }

        // don't use, probably buggy (but in degrees if you do)
        void setPosition(double position) { offset = position - getPosition() - offset; }

        // In degrees
        double getPosition(bool radians = false) { return lemlib::sanitizeAngle(m_motor->get_position() * m_ratio + offset, radians); }

        // both in degrees
        bool isAtPosition(double target, double tolerance = 5) {
            double error = lemlib::angleError(target, getPosition(), false);
            return std::fabs(error) < tolerance;
        }

        // In degrees
        void moveToPosition(double angle) { targetPose = lemlib::sanitizeAngle(angle, false); }
    protected:
        MotorPtr m_motor = nullptr;
        lemlib::PID m_pid;
        double kF;
        double m_ratio;
        double offset = idle;

        double targetPose = idle;

        void taskFunct() {
            int counter = 0;
            while (true) {
                double error = lemlib::angleError(targetPose, getPosition(), false);
                double voltage = m_pid.update(error) + kF * cos(getPosition(true));
                if (counter % 10 == 0) {
                    // std::printf("Arm angle: %f\n", getPosition());
                    // std::printf("Arm error: %f, voltage: %f\n", error, voltage);
                    std::printf("Arm angle: %f | %f\n", getPosition(), error);
                    std::printf("isArmUp: %d\n", isAtPosition(wall));
                }
                if (targetPose == idle && isAtPosition(idle, 10)) {
                    m_motor->move(0);
                    continue;
                }
                voltage = std::clamp(voltage, -127.0, 127.0);
                m_motor->move(voltage);
                counter++;
            }
        }
};

using ArmPtr = std::unique_ptr<Arm>;