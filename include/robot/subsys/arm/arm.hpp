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

        static double idle;
        static double wall;
        static double hang;

        void initialize() {
            m_motor->tare_position();
            pros::Task task([&]() { taskFunct(); });
            // moveToPosition(idle);
        }

        // don't use, probably buggy (but in degrees if you do)
        void setPosition(double position) { offset = position - getPosition() - offset; }

        // In degrees
        double getPosition(bool radians = false) {
            return lemlib::sanitizeAngle(m_motor->get_position() * m_ratio + offset, radians);
        }

        double getTargetPosition(bool radians = false) { return lemlib::sanitizeAngle(targetPose, false); }

        // both in degrees
        bool isAtPosition(double target, double tolerance = 5) {
            double error = lemlib::angleError(target, getPosition(false), false);
            return std::fabs(error) < tolerance;
        }

        // In degrees
        void moveToPosition(double angle) { targetPose = lemlib::sanitizeAngle(angle, false); }

        // In degrees
        void moveRelative(double angle) { moveToPosition(getPosition() + angle); }

        float get_temperature() const { return m_motor->get_temperature(); }
    protected:
        MotorPtr m_motor = nullptr;
        lemlib::PID m_pid;
        double kF;
        double m_ratio;
        double offset = idle;

        double targetPose = idle;

        void taskFunct();
};

using ArmPtr = std::unique_ptr<Arm>;