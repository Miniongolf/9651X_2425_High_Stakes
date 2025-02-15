#include "robot/chassis/chassis.hpp"

void CustomChassis::safeMoveToPoint(Waypoint wayPoint, bool async) {
    this->turnToPoint(wayPoint.x, wayPoint.y, wayPoint.turnTimeout, wayPoint.turnParams, async);
    this->moveToPoint(wayPoint.x, wayPoint.y, wayPoint.moveTimeout, wayPoint.moveParams, async);
}

void CustomChassis::safeMoveToPoint(float x, float y, int turnTimeout, int moveTimeout, TurnToPointParams turnParams, MoveToPointParams moveParams, bool async) {
    safeMoveToPoint(Waypoint(x, y, turnTimeout, moveTimeout, turnParams, moveParams), async);
}

void CustomChassis::safeMoveToPoint(float x, float y, int moveTimeout, MoveToPointParams moveParams, bool async) {
    safeMoveToPoint(Waypoint(x, y, 500, moveTimeout, {.forwards = moveParams.forwards}, moveParams), async);
}
