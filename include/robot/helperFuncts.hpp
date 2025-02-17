#pragma once
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

    void scoreAllianceStake();

    void hangTier3();
}