#include "robot/chassis/chassis.hpp"

void CustomChassis::safeMoveToPoint(Waypoint pathPoint, bool async) {
    this->turnToPoint(pathPoint.x, pathPoint.y, pathPoint.turnTimeout, pathPoint.turnParams, async);
    this->moveToPoint(pathPoint.x, pathPoint.y, pathPoint.moveTimeout, pathPoint.moveParams, async);
}

void CustomChassis::safeMoveToPoint(float x, float y, int turnTimeout, int moveTimeout, TurnToPointParams turnParams, MoveToPointParams moveParams, bool async) {
    safeMoveToPoint(Waypoint(x, y, turnTimeout, moveTimeout, turnParams, moveParams), async);
}
