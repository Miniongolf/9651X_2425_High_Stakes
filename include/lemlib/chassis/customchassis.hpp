#pragma once

#include "lemlib/util.hpp" // IWYU pragma: keep
#include "lemlib/timer.hpp" // IWYU pragma: keep
#include "lemlib/logger/logger.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "lemlib/catlibUtils.hpp" // IWYU pragma: keep

using namespace lemlib;

struct Waypoint {
        Waypoint(float x, float y, int turnTimeout = 500, int moveTimeout = 1500, TurnToPointParams turnParams = {},
                 MoveToPointParams moveParams = {})
            : x(x),
              y(y),
              turnTimeout(turnTimeout),
              moveTimeout(moveTimeout),
              turnParams(turnParams.forwards),
              moveParams(moveParams) {}

        Waypoint(Pose pose, int turnTimeout = 500, int moveTimeout = 1500, TurnToPointParams turnParams = {},
                 MoveToPointParams moveParams = {})
            : x(pose.x),
              y(pose.y),
              turnTimeout(turnTimeout),
              moveTimeout(moveTimeout),
              turnParams(turnParams),
              moveParams(moveParams) {}

        Waypoint(float x, float y, int moveTimeout, MoveToPointParams moveParams)
            : x(x),
              y(y),
              moveTimeout(moveTimeout),
              turnParams({.forwards = moveParams.forwards}),
              moveParams(moveParams) {}

        float x, y;
        int turnTimeout = 500, moveTimeout = 1500;
        TurnToPointParams turnParams = {};
        MoveToPointParams moveParams = {};
};

struct ChainTurnParams {
        float minSpeed;
        float maxSpeed;
        float swingAngle;
};

class CustomChassis : public lemlib::Chassis {
    public:
        CustomChassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                      OdomSensors sensors, DriveCurve* throttleCurve = &defaultDriveCurve,
                      DriveCurve* steerCurve = &defaultDriveCurve)
            : Chassis(drivetrain, linearSettings, angularSettings, sensors, throttleCurve, steerCurve) {}

        Eigen::Vector2d getEigenPoint(bool radians = false, bool standardPos = true) {
            lemlib::Pose pose = getPose(radians, standardPos);
            return Eigen::Vector2d(pose.x, pose.y);
        };

        void brake() {
            pros::MotorBrake stdMode = drivetrain.leftMotors->get_brake_mode();
            setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            drivetrain.leftMotors->move_velocity(0);
            drivetrain.rightMotors->move_velocity(0);
            pros::delay(30);
            setBrakeMode(stdMode);
        };

        void moveTimed(float throttle, float steering, int time, bool async = true);

        void moveDistance(float distance, int timeout, MoveToPointParams params, bool async = true);

        void swagTurnToHeading(float theta, DriveSide startLockide, int timeout, TurnToHeadingParams params,
                               bool async = true);
        void swagTurnToPoint(float x, float y, int timeout, TurnToHeadingParams params, bool async = true);

        void chainTurnToHeading(float theta, bool startForwards, bool endForwards, int timeout,
                                ChainTurnParams params = {}, bool async = true);

        void safeMoveToPoint(Waypoint waypoint, bool async = true);
        void safeMoveToPoint(float x, float y, int turnTimeout, int moveTimeout,
                             TurnToPointParams turnParams = {.earlyExitRange = 5}, MoveToPointParams moveParams = {},
                             bool async = true);
        void safeMoveToPoint(float x, float y, int moveTimeout, MoveToPointParams moveParams = {}, bool async = true);

        void pathInterp(std::vector<Waypoint> path, bool async = true);

        void followCurve(catlib::Curve curve, int timeout, float lookahead, float speedRatio = 1, bool forwards = true,
                         bool async = true);
};