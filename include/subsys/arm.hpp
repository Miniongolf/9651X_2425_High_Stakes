#pragma once

#include <memory>
#include <queue>
#include "lemlib/util.hpp"
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"

class AbstractArm {
    protected:
        AbstractArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
                    double length, double heightOffset = 0, double ratio = 1);
        double m_targetAngle = 0, m_angleOffset = 0;
        double m_length, m_heightOffset, m_jointOffset;
        std::unique_ptr<pros::Motor> m_motor;
        std::unique_ptr<pros::Rotation> m_rotSensor;
        lemlib::PID m_pid;

        double m_ratio;
    public:
        virtual void zeroRot() { m_rotSensor->set_position(0); }

        /**
         * @brief Get the angle of the arm, relative to the ground
         *
         * @return double angle in degrees
         */
        virtual double getAngle() { return double(m_rotSensor->get_angle()*m_ratio) / 100 + m_angleOffset; }

        /**
         * @brief Get the height of the arm
         *
         * @return double height
         */
        virtual double getHeight() { return angleToHeight(getAngle()); }

        /**
         * @brief Convert the angle of the arm to the height above the field at the end of the arm
         *
         * @param angle in degrees
         * @return double height
         */
        virtual double angleToHeight(double angle) {
            return m_heightOffset + m_length * std::sin(lemlib::degToRad(angle));
        }

        /**
         * @brief Move the arm to a specific angle
         *
         * @param angle in degrees
         */
        virtual void moveToAngle(double angle) { m_targetAngle = angle; }

        bool isAtTarget() { return std::fabs(this->getAngle() - m_targetAngle) < 2; }
};

class BaseArm : public AbstractArm {
    public:
        /**
         * @brief Construct a new Base Arm object
         *
         * @param motor unique ptr to a motor object
         * @param rotSensor unique ptr to a rotation sensor object
         * @param pid lemlib pid object
         * @param length length of the arm (in inches)
         * @param heightOffset height of the pivot (in inches)
         * @param ratio ratio of the rotation sensor
         */
        BaseArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
                double length, double heightOffset, double ratio);

        /**
         * @brief Destroy the Base Arm object
         */
        ~BaseArm() { m_task.remove(); }

        /**
         * @brief Resume the arm's task
         */
        void resumeTask() { m_task.resume(); }

        /**
         * @brief Suspend the arm's task
         */
        void suspendTask() { m_task.suspend(); }
    private:
        pros::Task m_task = pros::Task {[&] {
            double error;
            while (true) {
                pros::delay(10);

                error = lemlib::angleError(this->getAngle(), m_targetAngle, false);

                if (this->isAtTarget()) {
                    m_motor->move(0);
                } else {
                    m_motor->move(m_pid.update(error));
                }
            }
        }};
};

class TopArm : public AbstractArm {
    public:
        /**
         * @brief Construct a new Top Arm object
         *
         * @param motor unique ptr to a motor object
         * @param rotSensor unique ptr to a rotation sensor object
         * @param pid lemlib pid object
         * @param length length of the arm (in inches)
         * @param heightOffset height of the pivot (in inches)
         * @param baseArm unique ptr to a base arm object
         */
        TopArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
               double length, double heightOffset, std::unique_ptr<BaseArm> baseArm);

        /**
         * @brief Construct a new Top Arm object
         *
         * @param motor unique ptr to a motor object
         * @param rotSensor unique ptr to a rotation sensor object
         * @param pid lemlib pid object
         * @param length length of the arm (in inches)
         * @param heightOffset height of the pivot (in inches)
         * @param baseArm unique ptr to a base arm object
         * @param ratio ratio of the rotation sensor
         */
        TopArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
               double length, double heightOffset, double ratio, std::unique_ptr<BaseArm> baseArm);

        /**
         * @brief Destroy the Top Arm object
         */
        ~TopArm() { m_task.remove(); }

        /**
         * @brief Get the angle of the arm, relative to the ground. Takes the angle of the base arm into account.
         *
         * @return double angle in degrees
         */
        double getAngle() { return m_rotSensor->get_angle()*m_ratio + m_baseArm->getAngle(); }

        /**
         * @brief Resume the arm's task
         */
        void resumeTask() { m_task.resume(); }

        /**
         * @brief Suspend the arm's task
         */
        void suspendTask() { m_task.suspend(); }
    protected:
        std::unique_ptr<BaseArm> m_baseArm;
    private:
        pros::Task m_task = pros::Task {[&] {
            double error;

            while (true) {
                pros::delay(10);

                m_angleOffset = m_baseArm->getAngle();

                error = lemlib::angleError(this->getAngle(), m_targetAngle, false);

                if (this->isAtTarget()) {
                    m_motor->move(0);
                } else {
                    m_motor->move(m_pid.update(error));
                }
            }
        }};
};

struct DoubleArmPose {
        double bottomAngle, topAngle;
};

struct DoubleArmItem {};

struct DoubleArmMovement : DoubleArmItem {
    DoubleArmMovement(DoubleArmPose pose, int maxSpeed, int minSpeed, int slew)
        : endPose(pose), maxSpeed(maxSpeed), minSpeed(minSpeed), slew(slew) {};

    DoubleArmPose endPose;
    int maxSpeed, minSpeed, slew;
};

class DoubleArm {
    public:
        /**
         * @brief Construct a new Double Arm object
         *
         * @param baseArm unique ptr to a base arm object
         * @param topArm unique ptr to a top arm object
         */
        DoubleArm(std::unique_ptr<BaseArm> baseArm, std::unique_ptr<TopArm> topArm);

        /**
         * @brief Destroy the Double Arm object
         */
        ~DoubleArm() { m_task.remove(); }

        /**
         * @brief Resume the base and top arms' tasks
         */
        void resumeTasks();

        /**
         * @brief Suspend the base and top arms' tasks
         */
        void suspendTasks();

        /**
         * @brief Get the current pose of the arms
         *
         * @return DoubleArmPose object
         */
        DoubleArmPose getPose() { return {m_baseArm->getAngle(), m_topArm->getAngle()}; }

        /**
         * @brief Move the arms to a specific pose
         * @note This will add the motion to the queue
         *
         * @param pose DoubleArmPose object
         */
        void queuePose(DoubleArmPose pose) { m_poseQueue.push(pose); };

        /**
         * @brief Cancel the current motion
         * @note This will proceed to the next motion in the queue
         */
        void cancelCurrentMotion() { m_targetPose = this->getPose(); }

        /**
         * @brief Clear the motion queue
         * @note This will not stop the current motion
         */
        void clearMotionQueue() { m_poseQueue = {}; }

        /**
         * @brief Cancel all motions
         * @note This will clear the motion queue and stop the current motion
         */
        void cancelAllMotions() {
            this->cancelCurrentMotion();
            this->clearMotionQueue();
        }

        /**
         * @brief Move the arms to a specific pose, clearing the motion queue
         *
         * @param pose DoubleArmPose object
         */
        void forceMoveToPose(DoubleArmPose pose) {
            this->cancelAllMotions();
            this->queuePose(pose);
        }

        /**
         * @brief Check if the arms are in position
         *
         * @return true if the arms are in position
         * @return false if the arms are not in position
         */
        bool isAtTarget() { return m_baseArm->isAtTarget() && m_topArm->isAtTarget(); }
    protected:
        std::queue<DoubleArmPose> m_poseQueue;
        DoubleArmPose m_targetPose;

        std::unique_ptr<BaseArm> m_baseArm;
        std::unique_ptr<TopArm> m_topArm;
    private:
        pros::Task m_task = pros::Task {[&] {
            while (true) {
                pros::delay(10);
                if (!this->isAtTarget()) {
                    continue;
                } else {
                    m_poseQueue.pop();
                    if (!m_poseQueue.empty()) {
                        m_targetPose = m_poseQueue.front();
                    }
                }
            }
        }};
};