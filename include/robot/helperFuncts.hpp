#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp" // IWYU pragma: keep

struct PathPoint {
    public:
        /**
         * @brief Construct a new Path Point object
         * 
         * @param point
         * @param turnTimeout 
         * @param moveTimeout 
         * @param ttpParams
         * @param mtpParams
         */
        PathPoint(lemlib::Pose point, int turnTimeout, int moveTimeout, lemlib::TurnToPointParams ttpParams = {}, lemlib::MoveToPointParams mtpParams = {})
            : point(point), turnTimeout(turnTimeout), moveTimeout(moveTimeout), ttpParams(ttpParams), mtpParams(mtpParams) {}
    
    lemlib::Pose point;
    int turnTimeout, moveTimeout;
    lemlib::TurnToPointParams ttpParams;
    lemlib::MoveToPointParams mtpParams;
};

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
    void grabMogo(units::Pose pose, Time timeout, Length leadDist = 10_in, lemlib::MoveToPoseParams params = {}, double approachSpeed = 50, bool forwards = false, Length earlyExit = 5_in);

    /**
     * @brief Follow a path of points, moving linearly from point to point
     * 
     * @param path The vector of PathPoint objects
     * @param waitTime The time (in ms) to wait between movements/turns
     */
    void pathInterp(std::vector<PathPoint> path, int waitTime = 200);
}
