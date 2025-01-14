#pragma once

#include "util.hpp"
#include "robot/subsys/subsys.hpp"

extern Alliance robotAlliance;

extern Preroller preroller;
extern Hooks hooks;
extern Intake intake;

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