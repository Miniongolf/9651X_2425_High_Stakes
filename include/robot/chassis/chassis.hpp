#include <cmath> // IWYU pragma: keep
#include "lemlib/timer.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"

using namespace lemlib;

class CustomChassis : public lemlib::Chassis {
    public:
        CustomChassis(Drivetrain drivetrain, ControllerSettings linearSettings,
                ControllerSettings angularSettings, OdomSensors sensors,
                DriveCurve* throttleCurve = &defaultDriveCurve,
                DriveCurve* steerCurve = &defaultDriveCurve)
            : Chassis(drivetrain, linearSettings, angularSettings, sensors, throttleCurve, steerCurve) {}

        void swagTurnToHeading(float theta, DriveSide startLockide, int timeout, TurnToHeadingParams params, bool async);
        void swagTurnToPoint(float x, float y, int timeout, TurnToHeadingParams params, bool async);
};