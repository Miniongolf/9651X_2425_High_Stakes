#pragma once

#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "robodash/api.h"
#include "lemlib/api.hpp"
#include "subsys/intake.hpp"
#include "subsys/arm.hpp"
#include "constants.hpp"

extern bool isPtoActive;
extern bool isRedAlliance;

extern Intake intake;
extern Hooks hooks;
extern Conveyor conveyor;
extern Arm arm;

extern pros::adi::Pneumatics mogoMech;
extern pros::adi::Pneumatics ptoPiston;

extern pros::MotorGroup leftDrive, rightDrive;
extern pros::MotorGroup ptoLeftDrive, ptoRightDrive;

extern pros::IMU imu;
extern pros::Rotation horizRot, vertRot;
extern lemlib::TrackingWheel horizTracker, vertTracker;

extern lemlib::ControllerSettings lateralPID, angularPID;

extern lemlib::OdomSensors odom;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Drivetrain ptoDrivetrain;

extern lemlib::Chassis chassis;
extern lemlib::Chassis ptoChassis;
extern lemlib::Chassis* activeChassis;

extern rd::Selector selector;
//extern rd::Console console;