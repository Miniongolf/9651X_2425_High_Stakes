#include "robot/globals.hpp"
#include "autonFuncts.hpp"
#include "pros/motors.h"
#include <memory>

bool isRedAlliance = true;

pros::adi::Pneumatics mogoMech('B', false, false);
pros::adi::Pneumatics doinker('C', false, false);

pros::MotorGroup leftDrive({0, 0, 0}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({0, 0, 0}, pros::v5::MotorGears::blue);

pros::IMU imu(0);

pros::Rotation horizRot(0);
pros::Rotation vertRot(0);

lemlib::TrackingWheel horizTracker(
    &horizRot,
    2,
    0, // tune this
    1
);

lemlib::TrackingWheel vertTracker(
    &vertRot,
    lemlib::Omniwheel::NEW_275,
    0, // tune this
    1
);

lemlib::PID emptyLateralPID(5, 0, 0, 3, true);
lemlib::PID emptyAngularPID(2, 0, 0, 5, true);

lemlib::PID mogoLateralPID(5, 0, 0, 3, true);
lemlib::PID mogoAngularPID(2, 0, 0, 5, true);

lemlib::Drivetrain drivetrain(
    &leftDrive,
    &rightDrive,
    11.5, // measure this
    lemlib::Omniwheel::NEW_275,
    600,
    2
);

lemlib::OdomSensors odom(
    &vertTracker,
    nullptr,
    &horizTracker,
    nullptr,
    &imu
);

/**
 * @note take out 2 zeroes from timeouts (100 and 500)
 * increase timeouts when tuning PID
 */
lemlib::Chassis chassis(
    drivetrain,
    {emptyLateralPID.kP, emptyLateralPID.kI, emptyLateralPID.kD, emptyLateralPID.getWindupRange(), 1, 10000, 3, 50000, 5},
    {emptyAngularPID.kP, emptyAngularPID.kI, emptyAngularPID.kD, emptyAngularPID.getWindupRange(), 1, 10000, 3, 50000, 0},
    odom
);