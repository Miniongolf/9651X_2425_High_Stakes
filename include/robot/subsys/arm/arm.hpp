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
        void initialize() {
            m_motor->tare_position();
            pros::Task task([&](){taskFunct();});
        }

        // don't use, probably buggy (but in degrees if you do)
        void setPosition(double position) { offset = position - getPosition() - offset; }

        // In degrees
        double getPosition(bool radians = false) { return lemlib::sanitizeAngle(m_motor->get_position() * m_ratio + offset, radians); }
        double getTargetPosition(bool radians = false) { return lemlib::sanitizeAngle(targetPose); }

        // both in degrees
        bool isAtPosition(double target, double tolerance = 5) {
            double error = lemlib::angleError(target, getPosition(), false);
            return std::fabs(error) < tolerance;
        }

        // In degrees
        void moveToPosition(double angle) {
            idleNeedsPause = (angle == idle);
            targetPose = lemlib::sanitizeAngle(angle, false);
        }

        // In degrees
        void moveRelative(double angle) {
            moveToPosition(getPosition() + angle);
        }
    protected:
        MotorPtr m_motor = nullptr;
        lemlib::PID m_pid;
        double kF;
        double m_ratio;
        double offset = idle;

        double targetPose = idle;

        bool idleNeedsPause = false;

        void taskFunct() {
            int counter = 0;
            while (true) {
                pros::delay(10);
                double error = lemlib::angleError(targetPose, getPosition(), false);
                double voltage = m_pid.update(error) + kF * cos(getPosition(true));
                double maxVolt = 127;
                if (counter % 20 == 0) {
                    // std::printf("Arm angle: %f | %f\n", getPosition(), error);
                    // std::printf("isArmUp: %d\n", isAtPosition(wall));
                }
                if (targetPose == idle) {
                    // std::printf("Arm pause: %d\n", idleNeedsPause);
                    maxVolt = 10;
                    if (idleNeedsPause) {//} && isAtPosition(30, 20)) {
                        std::printf("ARM IDLE PAUSING\n");
                        m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        m_motor->move_velocity(0);
                        pros::delay(500);
                        idleNeedsPause = false;
                        m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                    }
                    if (isAtPosition(idle, 10)) {
                        m_motor->move(0);
                        continue;
                    }
                }
                voltage = std::clamp(voltage, -maxVolt, maxVolt);
                m_motor->move(voltage);
                counter++;
            }
        }
};

using ArmPtr = std::unique_ptr<Arm>;