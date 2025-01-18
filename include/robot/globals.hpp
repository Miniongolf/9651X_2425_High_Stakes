#pragma once

#include "subsys/clamp/clamp.hpp"
#include "util.hpp" // IWYU pragma: keep
#include "robot/constants.hpp"
#include "robot/subsys/subsys.hpp" // IWYU pragma: keep
#include "gamepad/gamepad.hpp" // IWYU pragma: keep

extern Alliance robotAlliance;

extern Preroller preroller;
extern Hooks hooks;
extern Intake intake;

extern Clamp mogoMech;

extern pros::MotorGroup leftDrive, rightDrive;

extern pros::IMU imu;
extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;

extern lemlib::Chassis chassis;