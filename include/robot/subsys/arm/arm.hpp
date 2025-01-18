#pragma once

#include "util.hpp"

class Arm {
    public:
        Arm(MotorPtr armMotor, lemlib::PID pid, double kF, double gearRatio)
            : m_motor(std::move(armMotor)),
              m_pid(pid),
              kF(kF),
              m_ratio(gearRatio) {
            m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        }

        void setPosition(Angle position) { offset = position - getPosition(); }

        Angle getPosition() { return from_stDeg(m_motor->get_position() * m_ratio) + offset; }

        void moveToPosition(Angle angle) {
            double error = lemlib::angleError(to_stDeg(angle), to_stDeg(getPosition()));
            double voltage = m_pid.update(error) + kF * units::cos(getPosition()).internal();
            voltage = std::clamp(voltage, -127.0, 127.0);
            m_motor->move(voltage);
        }
    protected:
        MotorPtr m_motor = nullptr;
        lemlib::PID m_pid;
        double kF;
        double m_ratio;
        Angle offset = 0_stRad;
};