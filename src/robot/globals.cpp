#include "robot/globals.hpp"

Alliance robotAlliance = Alliance::RED;

Arm arm( //
    makeMotor(-1, pros::MotorGears::green), //
    lemlib::PID(7, 0, 2, 0, true), //
    10, //
    1.0 / 6 //
);

Preroller preroller( //
    makeMotor(19, pros::MotorGears::green), //
    makeDistance(0) //
);

Hooks hooks( //
    makeMotor(-21, pros::MotorGears::blue), //
    makeOptical(0), //
    74, //
    {0, 19, 37, 56} //
);

Intake intake {
    PrerollerPtr(&preroller), //
    HooksPtr(&hooks), //
    ArmPtr(&arm) //
};

Clamp mogoMech(makePiston('A', false, false), makeDistance(11), &chassis);

pros::adi::Pneumatics doinker('C', false, false);

pros::MotorGroup leftDrive({17, -13, 15}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({-18, 16, -14}, pros::v5::MotorGears::blue);

pros::IMU imu(12);

pros::Rotation horizRot(0);
pros::Rotation vertRot(0);

lemlib::TrackingWheel horizTracker(&horizRot, 2,
                                   0, // tune this
                                   1);

lemlib::TrackingWheel vertTracker(&vertRot, 2.75,
                                  0, // tune this
                                  1);

lemlib::PID emptyLateralPID(6, 0, 0, 3, true);
lemlib::PID emptyAngularPID(2, 0, 9.5, 5, true);

lemlib::PID mogoLateralPID(5, 0, 2, 3, true);
lemlib::PID mogoAngularPID(1.5, 0, 10, 5, true);

lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive,
                              11.25, // measure this
                              lemlib::Omniwheel::NEW_275, 600, 2);

lemlib::OdomSensors odom(nullptr, nullptr, nullptr, nullptr, &imu);

/**
 * @note take out 2 zeroes from timeouts (100 and 500)
 * increase timeouts when tuning PID
 */
lemlib::Chassis chassis(drivetrain,
                        {emptyLateralPID.kP, emptyLateralPID.kI, emptyLateralPID.kD, emptyLateralPID.getWindupRange(),
                         1, 100, 3, 500, 5},
                        {emptyAngularPID.kP, emptyAngularPID.kI, emptyAngularPID.kD, emptyAngularPID.getWindupRange(),
                         1, 100, 3, 500, 0},
                        odom);