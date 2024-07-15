#pragma once

#include <cmath>
#include "lemlib/api.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "subsys/intake.hpp"
#include "subsys/arm.hpp"

extern Intake intake;
extern Hooks hooks;
extern Conveyor conveyor;

extern pros::adi::Pneumatics mogoMech;

extern pros::MotorGroup leftDrive, rightDrive;

extern pros::IMU imu;

extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::ControllerSettings lateralPID, angularPID;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

