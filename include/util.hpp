#pragma once

#include <atomic>
#include <memory>
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
// #include "units/Vector2D.hpp"
#include "robot/constants.hpp"

// Macro to define a new device unique pointer alias
#define NEW_DEVICE_PTR(device) using device##Ptr = std::unique_ptr<pros::device>;

// Motor
NEW_DEVICE_PTR(Motor);

[[nodiscard]] inline MotorPtr makeMotor(const std::int8_t port,
                                        const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid,
                                        const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::invalid) {
    return std::make_unique<pros::Motor>(port, gearset, encoder_units);
};

// MotorGroup
NEW_DEVICE_PTR(MotorGroup);

[[nodiscard]] inline MotorGroupPtr
makeMotorGroup(const std::vector<std::int8_t>& ports,
               const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid,
               const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::invalid) {
    return std::make_unique<pros::MotorGroup>(ports, gearset, encoder_units);
};

// IMU
NEW_DEVICE_PTR(Imu);

[[nodiscard]] inline ImuPtr makeImu(const std::int8_t port) { return std::make_unique<pros::Imu>(port); };

// Rotation
NEW_DEVICE_PTR(Rotation);

[[nodiscard]] inline RotationPtr makeRotation(const std::int8_t port) {
    return std::make_unique<pros::Rotation>(port);
};

// Optical
NEW_DEVICE_PTR(Optical);

[[nodiscard]] inline OpticalPtr makeOptical(const std::int8_t port) { return std::make_unique<pros::Optical>(port); };

// Distance
NEW_DEVICE_PTR(Distance);

[[nodiscard]] inline DistancePtr makeDistance(const std::int8_t port) {
    return std::make_unique<pros::Distance>(port);
};

// Piston
using PistonPtr = std::unique_ptr<pros::adi::Pneumatics>;

[[nodiscard]] inline PistonPtr makePiston(const std::uint8_t adi_port, bool start_extended, bool extended_is_low = false) {
    return std::make_unique<pros::adi::Pneumatics>(adi_port, start_extended, extended_is_low);
};

[[nodiscard]] inline PistonPtr makePiston(const pros::adi::ext_adi_port_pair_t port_pair, bool start_extended,
                                          bool extended_is_low = false) {
    return std::make_unique<pros::adi::Pneumatics>(port_pair, start_extended, extended_is_low);
};