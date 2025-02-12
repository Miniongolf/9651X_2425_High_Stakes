#pragma once

#include "lemlib/util.hpp" // IWYU pragma: keep
#include "lemlib/timer.hpp" // IWYU pragma: keep
#include "lemlib/logger/logger.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"

using namespace lemlib;

struct Waypoint {
    Waypoint(float x, float y, int turnTimeout, int moveTimeout, TurnToPointParams turnParams = {}, MoveToPointParams moveParams = {})
        : x(x), y(y), turnTimeout(turnTimeout), moveTimeout(moveTimeout), turnParams(turnParams.forwards), moveParams(moveParams) {}

    Waypoint(Pose pose, int turnTimeout, int moveTimeout, TurnToPointParams turnParams = {}, MoveToPointParams moveParams = {})
        : x(pose.x), y(pose.y), turnTimeout(turnTimeout), moveTimeout(moveTimeout), turnParams(turnParams), moveParams(moveParams) {}
    float x, y;
    int turnTimeout = 500, moveTimeout = 1000;
    TurnToPointParams turnParams = {};
    MoveToPointParams moveParams = {};
};

class CustomChassis : public lemlib::Chassis {
    public:
        CustomChassis(Drivetrain drivetrain, ControllerSettings linearSettings,
                ControllerSettings angularSettings, OdomSensors sensors,
                DriveCurve* throttleCurve = &defaultDriveCurve,
                DriveCurve* steerCurve = &defaultDriveCurve)
            : Chassis(drivetrain, linearSettings, angularSettings, sensors, throttleCurve, steerCurve) {}

        void moveDistance(float distance, int timeout, MoveToPointParams params, bool async);
        void swagTurnToHeading(float theta, DriveSide startLockide, int timeout, TurnToHeadingParams params, bool async);
        void swagTurnToPoint(float x, float y, int timeout, TurnToHeadingParams params, bool async);
        void safeMoveToPoint(Waypoint waypoint, bool async);
        void safeMoveToPoint(float x, float y, int turnTimeout, int moveTimeout, TurnToPointParams turnParams, MoveToPointParams moveParams, bool async);
        void pathInterp(std::vector<Waypoint> path, bool async);
};