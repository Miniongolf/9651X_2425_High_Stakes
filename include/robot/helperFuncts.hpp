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

    /**
     * @brief Grab a mogo at a given pose
     * 
     * @param pose The pose of the mogo
     * @param timeout The timeout of the first movement
     * @param leadDist The distance to lead the mogo by
     */
    void grabMogo(lemlib::Pose pose, Time timeout, Length leadDist = 10_in, lemlib::MoveToPoseParams params = {}, double approachSpeed = 50, bool forwards = false, Length earlyExit = 5_in);
}
