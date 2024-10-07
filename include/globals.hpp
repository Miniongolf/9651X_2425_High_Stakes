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

extern BaseArm baseArm;
extern TopArm topArm;
extern DoubleArm doubleArm;

extern pros::adi::Pneumatics mogoMech;
extern pros::adi::Pneumatics redirect;

extern pros::MotorGroup leftDrive, rightDrive;

extern pros::IMU imu;
extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::ControllerSettings lateralPID, angularPID;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;

extern lemlib::Chassis chassis;