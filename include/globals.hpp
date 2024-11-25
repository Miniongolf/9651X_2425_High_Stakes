#pragma once

#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "lemlib/api.hpp"
#include "subsys/intake.hpp"
#include "subsys/arm.hpp"
#include "constants.hpp"

extern bool isRedAlliance;

extern Intake intake;

extern Arm arm;

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