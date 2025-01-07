#pragma once

#include "util.hpp"

extern std::atomic<Alliance> robotAlliance;

extern pros::adi::Pneumatics mogoMech;
extern pros::adi::Pneumatics doinker;

extern pros::MotorGroup leftDrive, rightDrive;

extern pros::IMU imu;
extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::PID emptyLateralPID, emptyAngularPID;
extern lemlib::PID mogoLateralPID, mogoAngularPID;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;

extern lemlib::Chassis chassis;