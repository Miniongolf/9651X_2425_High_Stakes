#include "robot/chassis/chassis.hpp"

void CustomChassis::pathInterp(std::vector<Waypoint> path, bool async) {
    for (auto waypoint : path) {
        this->safeMoveToPoint(waypoint, async);
    }
}