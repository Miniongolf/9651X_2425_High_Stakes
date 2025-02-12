#include "robot/chassis/chassis.hpp"

void CustomChassis::pathInterp(std::vector<Waypoint> path, bool async) {
    for (int i = 0; i < path.size(); i++) {
        Waypoint pathPoint = path[i];
        this->safeMoveToPoint(pathPoint, async);
    }
}