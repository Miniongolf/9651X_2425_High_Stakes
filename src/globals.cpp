#include "globals.hpp"
#include "autonFuncts.hpp"
#include "pros/motors.h"
#include <memory>

bool isRedAlliance = true;

Intake intake(std::make_unique<pros::Motor>(20, pros::v5::MotorGears::blue));

BaseArm baseArm(
    std::make_unique<pros::Motor>(-17, pros::v5::MotorGears::green),
    std::make_unique<pros::Rotation>(7),
    {1, 0, 0},
    8.5,
    0,
    7
);

// TopArm topArm(
//     std::make_unique<pros::Motor>(-17, pros::v5::MotorGears::green),
//     std::make_unique<pros::Rotation>(7),
//     {0, 0, 0},
//     8.5,
//     0,
//     1,
//     std::make_unique(baseArm);
// );

pros::adi::Pneumatics mogoMech('B', true, true);


pros::MotorGroup leftDrive({-10, -1, 3}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({21, 5, -19}, pros::v5::MotorGears::blue);

pros::IMU imu(18);
pros::Optical optical(10);

pros::Rotation horizRot(6);
pros::Rotation vertRot(-4);

lemlib::TrackingWheel horizTracker(
    &horizRot,
    lemlib::Omniwheel::NEW_2,
    2.25,
    1
);

lemlib::TrackingWheel vertTracker(
    &vertRot,
    lemlib::Omniwheel::NEW_2,
    -1,
    1
);

lemlib::ControllerSettings lateralPID(
    10, // proportional gain (kP)
    0, // integral gain (kI), set to 0 to disable
    0, // derivative gain (kD)
    3, // integral anti-windup range, set to 0 to disable
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    5 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularPID(
    2.5, // proportional gain (kP)
    0, // integral gain (kI), set to 0 to disable
    0, // derivative gain (kD)
    3, // integral anti-windup range, set to 0 to disable
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    5 // maximum acceleration (slew)
);

lemlib::Drivetrain drivetrain(
    &leftDrive,
    &rightDrive,
    12.5,
    lemlib::Omniwheel::NEW_325,
    450,
    4
);

lemlib::OdomSensors odom(
    &vertTracker,
    nullptr,
    &horizTracker,
    nullptr,
    &imu
);

lemlib::Chassis chassis(
    drivetrain,
    lateralPID,
    angularPID,
    odom
);