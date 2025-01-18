#pragma once
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
     * @brief Suspends all subsystem tasks
     */
    void suspendTasks();

    /**
     * @brief Resumes all subsystem tasks
     */
    void resumeTasks();

    /**
     * @brief chassis.arcade() wrapper
     * 
     */
    void moveTimed(const double throttle, const double steering, const int time);

    /**
     * @brief Follow a path of points, moving linearly from point to point
     * 
     * @param path The vector of PathPoint objects
     * @param waitTime The time (in ms) to wait between movements/turns
     */
    void pathInterp(std::vector<PathPoint> path, int waitTime = 200);
}
