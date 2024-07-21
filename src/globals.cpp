#include "globals.hpp"

bool isPtoActive = false;

Intake intake(std::make_unique<pros::Motor>(-15, pros::v5::MotorGears::green));
Hooks hooks(std::make_unique<pros::Motor>(-20, pros::v5::MotorGears::green));
Conveyor conveyor(intake, hooks);

Arm arm(
    std::make_unique<pros::Motor>(3, pros::v5::MotorGears::blue),
    std::make_unique<pros::adi::Encoder>('C', 'D', false),
    1,
    std::make_unique<pros::Motor>(-19, pros::v5::MotorGears::blue),
    std::make_unique<pros::adi::Encoder>('E', 'F', false),
    1,
    lemlib::PID {5, 0, 3},
    -20
);

pros::adi::Pneumatics mogoMech('B', true, true);
pros::adi::Pneumatics ptoPiston('A', true, true);

pros::MotorGroup leftDrive({-1, -10, 3}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({21, 2, -19}, pros::v5::MotorGears::blue);

pros::MotorGroup ptoLeftDrive({-1, -10}, pros::v5::MotorGears::blue);
pros::MotorGroup ptoRightDrive({21, 2}, pros::v5::MotorGears::blue);

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

lemlib::Drivetrain ptoDrivetrain(
    &ptoLeftDrive,
    &ptoRightDrive,
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

lemlib::Chassis ptoChassis(
    ptoDrivetrain,
    lateralPID,
    angularPID,
    odom
);

lemlib::Chassis* activeChassis = &chassis;