#pragma once
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include <vector>


struct PathPoint {
    public:
        /**
         * @brief Construct a new Path Point object
         * 
         * @param point
         * @param turnTimeout 
         * @param moveTimeout 
         */
        PathPoint(lemlib::Pose point, int turnTimeout, int moveTimeout, lemlib::TurnToPointParams ttpParams = {}, lemlib::MoveToPointParams mtpParams = {})
            : point(point), turnTimeout(turnTimeout), moveTimeout(turnTimeout), ttpParams(ttpParams), mtpParams(mtpParams) {}
    
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
     * @brief Converts a pose relative to the robot to a global pose
     * 
     * @return lemlib::Pose 
     */
    lemlib::Pose relativeToGlobal(lemlib::Pose relativePose);

    /**
     * @brief Clamps a mogo and switches to the mogo PID constants
     */
    void clampMogo();

    /**
     * @brief Releases a mogo and switches to the empty PID constants
     */
    void releaseMogo();

    /**
     * @brief Toggles the mogo clamp and switches PID constants
     */
    void toggleMogo();

    /**
     * @brief chassis.arcade() wrapper
     * 
     */
    void moveTimed(const double throttle, const double steering, const int time);

    /**
     * @brief Suspends all subsystem tasks
     */
    void suspendTasks();

    /**
     * @brief Resumes all subsystem tasks
     */
    void resumeTasks();

    /**
     * @brief Moves the robot to a relative pose
     * 
     * @param relativePose the relative pose to move to
     * @param timeout the timeout for the movement
     * @param params LemLib MoveToPointParams object for movement
     * @param async whether the function should be run asynchronously. true by default
     */
    void moveRelative(lemlib::Pose relativePose, int timeout, lemlib::MoveToPointParams params = {}, bool async = true);

    /**
     * @brief Grabs a ring at a given pose
     *
     * @param ringPose the pose of the ring
     * @param params LemLib MoveToPoseParams object for approach movement
     * @param timeout1 timeout for the approach movement in ms
     * @param timeout2 timeout for the grab movement in ms (should be pretty short)
     * @param approachDist distance from the ring for the approach movement
     * @param holdRing whether to hold the ring for later or score it
     * @return true if a ring is detected in the intake, false otherwise
     * @note return data is not very useful for rings that are immediately scored
     */
    // bool grabRing(lemlib::Pose ringPose, int timeout1, int timeout2, double approachDist, lemlib::MoveToPoseParams params = {.minSpeed = 70}, bool holdRing = false);

    /**
     * @brief Grabs a mogo at a given pose using the mogo clamp
     *
     * @note This isn't for doinker rushes, do that yourself
     * 
     * @param mogoPose the pose of the mogo
     * @param timeout1 timeout for the approach movement in ms
     * @param timeout2 timeout for the grab movement in ms (should be pretty short)
     * @param approachDist distance from the mogo for the approach movement
     * @param params LemLib MoveToPointParams object for approach movements
     */
    // void grabMogo(lemlib::Pose mogoPose, int timeout1, int timeout2, double approachDist, lemlib::MoveToPointParams params = {.minSpeed = 70});

    /**
     * @brief Follow a path of points, moving linearly from point to point
     * 
     * @param path The vector of PathPoint objects
     * @param waitTime The time (in ms) to wait between movements/turns
     */
    void pathInterp(std::vector<PathPoint> path, int waitTime = 200);
}
