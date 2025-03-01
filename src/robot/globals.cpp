#include "robot/globals.hpp"
#include "pros/adi.hpp"

int hangLevel = 0;

Alliance robotAlliance = Alliance::BLUE;

Arm arm( //
    makeMotor(-1, pros::MotorGears::green), //
    lemlib::PID(7, 0, 2, 0, true), //
    15, //
    1.0 / 6 //
);

Preroller preroller( //
    makeMotor(9, pros::MotorGears::green), //
    makeDistance(5) //
);

Hooks hooks( //
    makeMotor(-20, pros::MotorGears::blue), //
    makeOptical(12), //
    makeRotation(11), //
    74, //
    {0, 55, 37, 18} //
);

Intake intake {
    PrerollerPtr(&preroller), //
    HooksPtr(&hooks), //
    ArmPtr(&arm) //
};

Clamp mogoMech(makePiston('H', false, false), makeDistance(11), &chassis);

pros::adi::Pneumatics doinker('G', false, false);

pros::MotorGroup leftDrive({13, -14, 15}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({-16, 18, -17}, pros::v5::MotorGears::blue);

pros::IMU imu(6);

pros::Rotation horizRot(-2);
pros::Rotation vertRot(19);

lemlib::TrackingWheel horizTracker(&horizRot, 2.05,
                                   -1, // tune this
                                   1);

lemlib::TrackingWheel vertTracker(&vertRot, 2.75,
                                  0, // tune this
                                  1);

lemlib::PID emptyLateralPID(7, 0, 20, 3, true);
lemlib::PID emptyAngularPID(1.6, 0, 8, 5, true);

lemlib::PID mogoLateralPID(8, 0, 5, 3, true);
lemlib::PID mogoAngularPID(2, 0, 15, 5, true);

lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive,
                              11.25, // measu`re this
                              lemlib::Omniwheel::NEW_275, 600, 5);

lemlib::OdomSensors odom(&vertTracker, nullptr, &horizTracker, nullptr, &imu);

/**
 * @note take out 2 zeroes from timeouts (100 and 500)
 * increase timeouts when tuning PID
 */
CustomChassis chassis( //
    drivetrain,
    {emptyLateralPID.kP, emptyLateralPID.kI, emptyLateralPID.kD, emptyLateralPID.getWindupRange(), 1, 100, 3, 500, 5},
    {emptyAngularPID.kP, emptyAngularPID.kI, emptyAngularPID.kD, emptyAngularPID.getWindupRange(), 1, 100, 3, 500, 20},
    odom //
);