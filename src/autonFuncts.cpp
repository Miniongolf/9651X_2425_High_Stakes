#include "autonFuncts.hpp"

namespace auton {
void tunePID(bool hasMogo) {
    std::printf("Lateral Gains (%f, %f, %f)", chassis.lateralPID.kP, chassis.lateralPID.kI, chassis.lateralPID.kD);
    std::printf("Angular Gains (%f, %f, %f)", chassis.angularPID.kP, chassis.angularPID.kI, chassis.angularPID.kD);
    hasMogo ?  robot::clampMogo() : robot::releaseMogo();
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    chassis.turnToHeading(135, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    chassis.turnToHeading(90, 4000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
    chassis.moveToPoint(36, chassis.getPose().y, 5000);
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
}
} // namespace auton