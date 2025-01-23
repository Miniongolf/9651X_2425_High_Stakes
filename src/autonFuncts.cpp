#include "autonFuncts.hpp"
#include "robot/helperFuncts.hpp"

namespace auton {
void tunePID(bool hasMogo) {
    mogoMech.setState(hasMogo);
    std::printf("Lateral Gains (%f, %f, %f)", chassis.lateralPID.kP, chassis.lateralPID.kI, chassis.lateralPID.kD);
    std::printf("Angular Gains (%f, %f, %f)", chassis.angularPID.kP, chassis.angularPID.kI, chassis.angularPID.kD);
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    // Long turn
    chassis.turnToHeading(135, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    // Short turn
    chassis.turnToHeading(90, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    // Lateral
    chassis.moveToPoint(36, chassis.getPose().y, 5000, {}, false);
    pros::delay(100);
    robot::printPose();
}

void autoTestStuffs() {
    chassis.setPose(0, 0, 0);
    robot::grabMogo(
        {48, 48, 90}, 1000_msec, 10_in, {},
        50, false, 5_in
    );
}
} // namespace auton