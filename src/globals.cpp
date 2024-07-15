#include "globals.hpp"

Intake intake(std::make_unique<pros::Motor>(-15));
Hooks hooks(std::make_unique<pros::Motor>(-20));
Conveyor conveyor(intake, hooks);

pros::adi::Pneumatics mogoMech('B', false);
pros::adi::Pneumatics ptoPiston('A', false);

pros::MotorGroup leftDrive({-1, -20, 3});
pros::MotorGroup rightDrive({21, 2, -19});

pros::IMU imu(18);

pros::Rotation horizRot(5);
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
    3, // derivative gain (kD), set to 3
    3, // integral anti-windup range, set to 0 to disable
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    5 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularPID(
    2, // proportional gain (kP)
    0, // integral gain (kI), set to 0 to disable
    10, // derivative gain (kD), set to 3
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
    2
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