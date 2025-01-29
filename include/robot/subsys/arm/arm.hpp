#pragma once

#include "lemlib/util.hpp"
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

        constexpr static Angle idle = -50_stDeg;
        constexpr static Angle wall = 60_stDeg;

        void initialize() {
            setPosition(idle);
            pros::Task task([&](){taskFunct();});
        }

        void setPosition(Angle position) { offset = position - getPosition(); }

        Angle getPosition() { return from_stDeg(m_motor->get_position() * m_ratio) + offset; }

        bool isAtPosition(Angle target, AngleRange tolerance = 1.5_stDeg) {
            double error = lemlib::angleError(to_stRad(target), to_stRad(getPosition()));
            return std::fabs(error) < to_stRad(tolerance);
        }

        void moveToPosition(Angle angle) { targetPose = angle; }
    protected:
        MotorPtr m_motor = nullptr;
        lemlib::PID m_pid;
        double kF;
        double m_ratio;
        Angle offset = 0_stDeg;

        Angle targetPose = 0_stDeg;

        void taskFunct() {
            while (true) {
                if (targetPose == idle && isAtPosition(idle, 10_stDeg)) {
                    m_motor->move(0);
                    continue;
                }
                double error = lemlib::angleError(to_stRad(targetPose), to_stRad(getPosition()));
                double voltage = m_pid.update(error) + kF * units::cos(getPosition()).internal();
                voltage = std::clamp(voltage, -127.0, 127.0);
                m_motor->move(voltage);
            }
        }
};

using ArmPtr = std::unique_ptr<Arm>;