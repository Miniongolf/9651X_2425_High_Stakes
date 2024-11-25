#pragma once

#include <cmath>
#include <memory>
#include <atomic>
#include <utility>
#include "lemlib/util.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"

namespace armPositions {
constexpr double standby = 171;
constexpr double load = 183;
constexpr double extendedLimit = 45;
constexpr double wallStake = 50;
constexpr double allianceStake = 340;
constexpr double mogoTip = 20;
} // namespace armPositions

class PIDf : public lemlib::PID {
    public:
        /**
         * @brief Construct a new PIDf object
         *
         * @param kP
         * @param kI
         * @param kD
         * @param kF
         * @param range
         * @param slew
         * @param windupRange
         * @param signFlipReset
         */
        PIDf(double kP, double kI, double kD, double kF = 0, std::pair<double, double> range = {-127, 127},
             int slew = 0, float windupRange = 0, bool signFlipReset = false)
            : lemlib::PID(kP, kI, kD, windupRange, signFlipReset),
              m_kF(kF),
              m_range(range),
              m_slew(slew) {}

        /**
         * @brief Set the constants of the PIDf controller
         *
         * @param kP
         * @param kI
         * @param kD
         * @param kF
         */
        void setConstants(double kP, double kI, double kD, double kF) {
            this->kP = kP;
            this->kI = kI;
            this->kD = kD;
            m_kF = kF;
        }

        /**
         * @brief Get an output from the PIDf controller
         *
         * @param error target - current value
         * @param feedforward
         * @return double
         */
        double update(double error, double feedforward) {
            double output = lemlib::PID::update(error) + m_kF * feedforward;
            lemlib::slew(output, m_prevOutput, m_slew);
            output = std::clamp(output, m_range.first, m_range.second);
            m_prevOutput = output;
            return output;
        }
    protected:
        double m_kF;
        double m_slew;
        std::pair<double, double> m_range;
        double m_prevOutput = 0;
};

class Arm {
    public:
        /**
         * @brief Construct a new Arm object
         *
         * @param motor unique ptr to a motor object
         * @param rotSens unique ptr to a rotation sensor object
         * @param piston unique ptr to a pneumatics object
         * @param pid LemLib PID controller object
         */
        Arm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSens,
            std::unique_ptr<pros::adi::Pneumatics> piston, PIDf pid)
            : m_motor(std::move(motor)),
              m_rotSens(std::move(rotSens)),
              m_piston(std::move(piston)),
              m_pid(pid) {
            m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        };

        /**
         * @brief Destroy the Arm object
         */
        ~Arm() { m_task.remove(); }

        /**
         * @brief Suspend the arm task
         */
        void suspendTask() { m_task.suspend(); }

        /**
         * @brief Resume the arm task
         */
        void resumeTask() { m_task.resume(); }

        /**
         * @brief Set the extension of the arm
         *
         * @param extended whether to extend the arm
         */
        void setExtension(const bool extended) {
            if (extended) m_piston->extend();
            else m_piston->retract();
        }

        /**
         * @brief Toggle the extension of the arm
         */
        void toggle() {
            if (m_piston->is_extended()) this->retract();
            else this->extend();
        }

        /**
         * @brief Retract the slide
         */
        void retract() { m_piston->retract(); }

        /**
         * @brief Extend the slide
         */
        void extend() {
            if (!this->canExtend()) return;
            m_piston->extend();
            if (targetAngle < armPositions::extendedLimit) { targetAngle = armPositions::extendedLimit; }
            if (bufferAngle < armPositions::extendedLimit) { targetAngle = armPositions::extendedLimit; }
        }

        /**
         * @brief Returns whether or not the arm can extend without going out of expansion limits
         */
        bool canExtend() { return !(this->getAngle() < armPositions::extendedLimit || this->getAngle() > 230); }

        /**
         * @brief Returns whether or not a movement is already buffered
         */
        bool hasBuffer() { return this->bufferAngle != this->targetAngle; }

        /**
         * @brief Returns the angle of the arm
         */
        double getAngle() { return lemlib::sanitizeAngle(m_rotSens->get_position() / 100.0, false); }

        /**
         * @brief Checks if the arm is at a specific angle
         * @param angle the angle to check against
         */
        bool isAtPosition(const double angle) {
            return std::fabs(lemlib::angleError(angle, this->getAngle(), false)) < 1;
        }

        /**
         * @brief Checks if the arm is at the target angle
         * @param checkBuffer whether to check only the primary target or both the target and buffer
         */
        bool isAtTarget(const bool checkBuffer = true) {
            bool atTarget = this->isAtPosition(targetAngle);
            if (checkBuffer) {
                bool atBuffer = this->isAtPosition(bufferAngle);
                return atTarget && atBuffer;
            }
            return atTarget;
        }

        /**
         * @brief Waits until the arm is at the target angle
         * @note Blocking operation
         */
        void waitUntilSettled() {
            while (!this->isAtTarget()) pros::delay(10);
        }

        /**
         * @brief Move the arm to a specific angle
         *
         * @param angle target angle to move the arm to in degrees (standard angle system)
         * @param force whether to immediately force the new movement
         * @param async whether to run the movement asynchronously
         *
         * @note This command will override the previous buffered angle.
         * @note If `force` is false, the movement will be buffered until the arm completes its current movement.
         */
        void moveToAngle(double angle, const bool force = true, const bool async = true) {
            angle = lemlib::sanitizeAngle(angle, false);
            if (angle > armPositions::load + 5 && angle < 270) { return; }
            if (m_piston->is_extended() && (angle < armPositions::extendedLimit || angle > 250)) {
                angle = armPositions::extendedLimit;
            }

            this->bufferAngle = angle;
            if (force) this->targetAngle = angle;
            if (!async) { this->waitUntilSettled(); }
        }

        void changeAngle(double angle) { this->moveToAngle(this->getAngle() + angle, true); }
        
        std::atomic<double> targetAngle = armPositions::load, bufferAngle = armPositions::load;

        std::unique_ptr<pros::Motor> m_motor;
        std::unique_ptr<pros::adi::Pneumatics> m_piston;
    protected:
        std::unique_ptr<pros::Rotation> m_rotSens;
        PIDf m_pid;


        pros::Task m_task {[&] {
            int counter = 0;
            int voltage = 0, lastVoltage = 0;

            targetAngle = armPositions::load;
            bufferAngle = armPositions::load;
            while (true) {
                pros::delay(10);

                if (m_piston->is_extended() && !this->canExtend()) { m_piston->retract(); }

                double cwAngleError = lemlib::angleError(this->getAngle(), this->targetAngle, false,
                                                         AngularDirection::CW_CLOCKWISE),
                       ccwAngleError = lemlib::angleError(this->getAngle(), this->targetAngle, false,
                                                          AngularDirection::CCW_COUNTERCLOCKWISE);

                double smaller, larger;
                if (std::fabs(cwAngleError) < std::fabs(ccwAngleError)) {
                    smaller = cwAngleError;
                    larger = ccwAngleError;
                } else {
                    smaller = ccwAngleError;
                    larger = cwAngleError;
                }

                double midPoint = lemlib::sanitizeAngle(this->getAngle() - smaller / 2, false);
                double usedError = ((midPoint > armPositions::load + 5) && (midPoint < 300)) ? larger : smaller;

                if (this->targetAngle == armPositions::load && this->getAngle() > 120 && this->getAngle() > 200) {
                    voltage = 0;
                } else if (this->isAtPosition(armPositions::load)) {
                    voltage = m_pid.update(-usedError, 0);
                } else {
                    voltage = m_pid.update(-usedError, std::cos(lemlib::degToRad(this->getAngle())));
                }

                m_motor->move(voltage);

                if (counter % 20 == 0) {
                    double target = this->targetAngle;
                    // std::printf("Arm position: %f\n", this->getAngle());
                    // std::printf("Arm positions: %d %d\n", (midPoint > armPositions::load + 5) && (midPoint < 300), usedError==smaller);
                    counter = 0;
                }
                counter++;
            }
        }};
};