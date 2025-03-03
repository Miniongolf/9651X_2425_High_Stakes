#pragma once

#include "pros/adi.hpp"
#include "util.hpp" // IWYU pragma: keep
#include "gamepad/gamepad.hpp" // IWYU pragma: keep
#include "robot/constants.hpp"
#include "lemlib/chassis/customchassis.hpp" // IWYU pragma: keep
#include "robot/subsys/subsys.hpp" // IWYU pragma: keep

extern int hangLevel;

extern Alliance robotAlliance;

extern Arm arm;

extern Preroller preroller;
extern Hooks hooks;
extern Intake intake;

extern Clamp mogoMech;

extern pros::adi::Pneumatics doinker;

extern pros::MotorGroup leftDrive, rightDrive;

extern pros::IMU imu;
extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;

extern CustomChassis chassis;