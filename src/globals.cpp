#include "globals.hpp"
#include "autonFuncts.hpp"
#include "pros/motors.h"
#include <memory>

bool isRedAlliance = true;

Intake intake(
    std::make_unique<pros::Motor>(20, pros::v5::MotorGears::blue),
    std::make_unique<pros::Motor>(19, pros::v5::MotorGears::green)
    
);

Arm arm(
    std::make_unique<pros::Motor>(-9, pros::v5::MotorGears::green),
    std::make_unique<pros::Rotation>(-10),
    std::make_unique<pros::adi::Pneumatics>('A', false, false),
    {5, 0.1, 12, 20, {-127, 127}, 5, 3, true}
    // {0, 0, 0, 20, {-127, 127}, 5, 3, true}
    // {0, 0, 0}
);

pros::adi::Pneumatics mogoMech('B', false, false);
pros::adi::Pneumatics doinker('C', false, false);


pros::MotorGroup leftDrive({-12, -13, 14}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({2, 3, -4}, pros::v5::MotorGears::blue);

pros::IMU imu(5);

pros::Rotation horizRot(1);
pros::Rotation vertRot(-11);

lemlib::TrackingWheel horizTracker(
    &horizRot,
    lemlib::Omniwheel::NEW_2,
    1.5,
    1
);

lemlib::TrackingWheel vertTracker(
    &vertRot,
    lemlib::Omniwheel::NEW_2,
    -0.25,
    1
);

lemlib::PID emptyLateralPID(7, 0, 15, 3, true);
lemlib::PID emptyAngularPID(2, 0, 12, 5, true);

lemlib::PID mogoLateralPID(9, 0, 20, 3, true);
lemlib::PID mogoAngularPID(2.5, 0, 15, 5, true);

lemlib::Drivetrain drivetrain(
    &leftDrive,
    &rightDrive,
    11.5,
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