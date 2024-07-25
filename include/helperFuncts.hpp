#pragma once
#include "globals.hpp"
#include "lemlib/chassis/odom.hpp"

namespace robot {
void suspendTasks();
void resumeTasks();
void setPTO(bool state);
void chassisGrabRing(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {});
void chassisGrabMogo(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {});

/* Chassis Functions */

/*
void arcadeDrive(int throttle, int turn);

void chassisTurnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true);
void chassisTurnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
void chassisSwingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params = {}, bool async = true);
void chassisSwingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params = {}, bool async = true);
*/
//void chassisMoveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = true);

}