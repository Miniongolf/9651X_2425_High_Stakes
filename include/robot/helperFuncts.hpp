#pragma once
#include "pros/motors.h"
#include "robot/globals.hpp" // IWYU pragma: keep

namespace robot {
    /**
     * @brief Prints the current pose of the robot
     */
    void printPose();

    /**
     * @brief chassis.arcade() wrapper
     * 
     */
    void moveTimed(const double throttle, const double steering, const int time);

    void safeGrabMogo(float x, float y, int timeout);

    void scoreAllianceStake(pros::motor_brake_mode_e brakeMode = pros::E_MOTOR_BRAKE_COAST);

    void hangTier3();
}