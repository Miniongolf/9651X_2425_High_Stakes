#include "robot/helperFuncts.hpp"
#include "pros/rtos.hpp"

namespace robot {
void printPose() {
    std::cout << "Chassis pose &t=" << pros::millis() << ": " << format_as(chassis.getPose()) << "\n";
    std::printf("Pose: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

void moveTimed(const double throttle, const double steering, const int time) {
    chassis.arcade(throttle, steering, true, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

void pathInterp(std::vector<PathPoint> path, int waitTime) {
    for (int i = 0; i < path.size(); i++) {
        PathPoint pathPoint = path[i];
        chassis.turnToPoint(pathPoint.point.x, pathPoint.point.y, pathPoint.turnTimeout, pathPoint.ttpParams, false);
        pros::delay(waitTime);
        robot::printPose();

        pathPoint.mtpParams.forwards = pathPoint.ttpParams.forwards;

        chassis.moveToPoint(pathPoint.point.x, pathPoint.point.y, pathPoint.moveTimeout, pathPoint.mtpParams, false);
        pros::delay(waitTime);
        robot::printPose();
    }
}
} // namespace robot