#include "subsys/arm.hpp"

// Abstract Arm
AbstractArm::AbstractArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
                         double length, double heightOffset, double ratio)
    : m_motor(std::move(motor)),
      m_rotSensor(std::move(rotSensor)),
      m_pid(pid),
      m_length(length),
      m_heightOffset(heightOffset),
      m_ratio(ratio) {}

// Base Arm
BaseArm::BaseArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
                 double length, double heightOffset, double ratio)
    : AbstractArm(std::move(motor), std::move(rotSensor), pid, length, heightOffset, ratio) {}

// Top Arm
TopArm::TopArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
               double length, double heightOffset, double ratio, std::unique_ptr<BaseArm> baseArm)
    : AbstractArm(std::move(motor), std::move(rotSensor), pid, length, heightOffset, ratio),
      m_baseArm(std::move(baseArm)) {}

TopArm::TopArm(std::unique_ptr<pros::Motor> motor, std::unique_ptr<pros::Rotation> rotSensor, lemlib::PID pid,
               double length, double heightOffset, std::unique_ptr<BaseArm> baseArm)
    : AbstractArm(std::move(motor), std::move(rotSensor), pid, length, heightOffset),
      m_baseArm(std::move(baseArm)) {}

// Double Arm
DoubleArm::DoubleArm(std::unique_ptr<BaseArm> baseArm, std::unique_ptr<TopArm> topArm)
    : m_baseArm(std::move(baseArm)),
      m_topArm(std::move(topArm)) {}

void DoubleArm::resumeTasks() {
    m_task.resume();
    m_baseArm->resumeTask();
    m_topArm->resumeTask();
}

void DoubleArm::suspendTasks() {
    m_task.suspend();
    m_baseArm->suspendTask();
    m_topArm->suspendTask();
}