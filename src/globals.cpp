#include "globals.hpp"
#include "autonFuncts.hpp"

bool isPtoActive = false;
bool isRedAlliance = true;

Intake intake(std::make_unique<pros::Motor>(-15, pros::v5::MotorGears::green));
Hooks hooks(std::make_unique<pros::Motor>(-20, pros::v5::MotorGears::green));
Conveyor conveyor(intake, hooks, std::make_unique<pros::Optical>(10));

Arm arm(
    std::make_unique<pros::Motor>(3, pros::v5::MotorGears::blue),
    std::make_unique<pros::Rotation>(6),
    -0.25,
    std::make_unique<pros::Motor>(-19, pros::v5::MotorGears::blue),
    std::make_unique<pros::Rotation>(14),
    -0.25,
    lemlib::PID {5, 0, 3},
    -20
);

pros::adi::Pneumatics ptoPiston('A', isPtoActive, false);
pros::adi::Pneumatics mogoMech('B', true, true);

pros::MotorGroup leftDrive({-1, -9, 3}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({21, 2, -19}, pros::v5::MotorGears::blue);

pros::MotorGroup ptoLeftDrive({-1, -9}, pros::v5::MotorGears::blue);
pros::MotorGroup ptoRightDrive({21, 2}, pros::v5::MotorGears::blue);

pros::IMU imu(18);
pros::Optical optical(10);

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
    85, // proportional gain (kP)
    0, // integral gain (kI), set to 0 to disable
    15, // derivative gain (kD)
    3, // integral anti-windup range, set to 0 to disable
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularPID(
    3, // proportional gain (kP)
    0, // integral gain (kI), set to 0 to disable
    20, // derivative gain (kD)
    3, // integral anti-windup range, set to 0 to disable
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

lemlib::Drivetrain drivetrain(
    &leftDrive,
    &rightDrive,
    12.5,
    lemlib::Omniwheel::NEW_275,
    450,
    4
);

lemlib::Drivetrain ptoDrivetrain(
    &ptoLeftDrive,
    &ptoRightDrive,
    12.5,
    lemlib::Omniwheel::NEW_275,
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

lemlib::ExpoDriveCurve throttleCurve(5, 20, 1);
lemlib::ExpoDriveCurve steerCurve(5, 10, 1);

lemlib::Chassis chassis(
    drivetrain,
    lateralPID,
    angularPID,
    odom,
    &throttleCurve,
    &steerCurve
);

lemlib::Chassis ptoChassis(
    ptoDrivetrain,
    lateralPID,
    angularPID,
    odom,
    &throttleCurve,
    &steerCurve
);

lemlib::Chassis* activeChassis = &chassis;