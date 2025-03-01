#include "autonFuncts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "robot/globals.hpp"
#include "robot/helperFuncts.hpp"
#include "robot/subsys/intake/intake.hpp"

lemlib::Pose doinkerClampPose() {
    chassis.waitUntilDone();
    lemlib::Pose relDoink = lemlib::Pose {23, 5}.rotate(chassis.getPose(true, true).theta);
    std::printf("RelX: %f, RelY: %f\n", relDoink.x, relDoink.y);
    return {chassis.getPose().x + relDoink.x, chassis.getPose().y + relDoink.y};
}

namespace auton {
int split = 0;
Time startTime = 0_msec;

void resetSplit() {
    split = 0;
    startTime = from_msec(pros::millis());
}

void nextSplit(std::string name) {
    split++;
    Time elapsedTime = from_msec(pros::millis()) - startTime;
    std::cout << "Split " << split << ": " << name << " @ " << elapsedTime << "\n";
}

void tunePID(bool hasMogo) {
    mogoMech.setState(hasMogo);
    std::printf("Lateral Gains (%f, %f, %f)", chassis.lateralPID.kP, chassis.lateralPID.kI, chassis.lateralPID.kD);
    std::printf("Angular Gains (%f, %f, %f)", chassis.angularPID.kP, chassis.angularPID.kI, chassis.angularPID.kD);
    chassis.setPose(0, 0, 0);
    pros::delay(200);
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

void ringRush() {
    // Side flipper
    int s = robotAlliance == Alliance::RED ? 1 : -1;

    chassis.setPose(-49, 26, 70.5);
    doinker.extend();
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveTimed(127, 0, 750, false);
    chassis.brake();
    chassis.swingToPoint(-24, 24, lemlib::DriveSide::RIGHT, 1000, {.forwards = false});
    chassis.moveToPoint(-21, 18, 1000, {.forwards = false, .maxSpeed = 70}, false);
    chassis.moveTimed(-60, 0, 800);
    mogoMech.clamp(true);
    chassis.waitUntilDone();
    chassis.brake();
    pros::delay(700);
    doinker.retract();
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    pros::delay(750);
    chassis.safeMoveToPoint(-24, 48, 1000, {.maxSpeed = 60});
    chassis.safeMoveToPoint(-54, 54, 1000);
    // chassis.brake(pros::E_MOTOR_BRAKE_COAST);
}

void autoTestStuffs() {
    chassis.setPose(0, 0, 0);
    chassis.followCurve({{0, 0}, {0, 24}, {24, 0}, {24, 24}}, 3000, 6, 1, true, false);
    chassis.followCurve({chassis.getEigenPoint(), {24, 47}, {-65, 32}, {0, 0}}, 3000, 6);
    chassis.waitUntilDone();
}

void blueGoalRush() {
    chassis.setPose(55, -63, -65.5);
    resetSplit();

    doinker.extend();
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    // Goal rush
    chassis.moveTimed(127, 0, 650, false);
    robot::printPose();
    chassis.moveToPoint(55, -70, 1000, {.forwards = false, .minSpeed = 70}, false);
    lemlib::Pose rushedGoalPose = doinkerClampPose();
    nextSplit("Goal rush pull back");
    // Grab rushed goal
    chassis.turnToHeading(chassis.getPose().theta + 35, 1000, {.minSpeed = 90, .earlyExitRange = 5});
    chassis.moveTimed(90, 0, 180, false);
    doinker.retract();
    chassis.moveTimed(-60, 0, 200, false);
    chassis.safeMoveToPoint(rushedGoalPose.x, rushedGoalPose.y, 1000, 1000,
                            {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE},
                            {.forwards = false, .maxSpeed = 70}, false);
    mogoMech.clamp(true);
    nextSplit("Grab rushed goal");
    intake.setMode(Intake::modes::CONTINUOUS);
    chassis.turnToPoint(70, -80, 750, {}, false);
    pros::delay(600);
    intake.setMode(Intake::modes::INDEX);
    intake.forwards();
    // Grab corner ring
    chassis.moveTimed(80, 0, 1000);
    pros::delay(300);
    mogoMech.release(false);
    chassis.moveTimed(127, 0, 1000, false);
    nextSplit("Corner ring");
    chassis.swingToPoint(60, 0, lemlib::DriveSide::RIGHT, 1000, {}, false);
    intake.setMode(Intake::modes::HOLD);
    intake.reverse();
    chassis.moveToPoint(60, 5, 2000, {.earlyExitRange = 16}, false);
    chassis.moveToPoint(60, 5, 1000, {.maxSpeed = 60, .minSpeed = 0}, false);
    chassis.brake();
    chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 1000);
    chassis.moveTimed(-60, 0, 300);
    chassis.setPose(63, 0, chassis.getPose().theta);
    robot::scoreAllianceStake();
    nextSplit("Alliance stake");
    chassis.turnToHeading(-90, 1000);
    chassis.moveTimed(50, 0, 500);

    // chassis.swingToHeading(-135, lemlib::DriveSide::LEFT, 1000, {}, false);
    // chassis.moveToPose(24, -12, -90, 1000);
    // chassis.safeMoveToPoint(12, -12, 1000, {}, false);
    // chassis.turnToHeading(-60, 1000, {}, false);
    // doinker.extend();
    // chassis.moveTimed(-70, 0, 750);
    // mogoMech.clamp(true);
    // intake.setMode(Intake::modes::CONTINUOUS);
    // intake.forwards();
    // chassis.moveToPoint(24, -24, 500);
    // chassis.moveTimed(50, 0, 500);
    nextSplit("Touch bar");

}

void goalRush() {
    // Side flipper
    int s = robotAlliance == Alliance::RED ? 1 : -1;

    chassis.setPose(-52 * s, -31, 116);
    doinker.extend();
    intake.setMode(Intake::modes::HOLD);
    intake.forwards();
    chassis.moveTimed(127, 0, 580, false);
    // chassis.brake(pros::E_MOTOR_BRAKE_COAST);
    chassis.safeMoveToPoint(-38 * s, -36, 1600, {.forwards = false});
    chassis.turnToHeading(180 * s, 1000);
    doinker.retract();
    chassis.safeMoveToPoint(-11 * s, -44, 1000, {.forwards = false, .maxSpeed = 70}, false);
    robot::printPose();
    // chassis.swingToPoint(-15, -34, lemlib::DriveSide::RIGHT, 1000, {.forwards=false});
    // chassis.moveTimed(-60, 0, 800);
    mogoMech.clamp(true);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.safeMoveToPoint(-36 * s, -59, 1000, {.forwards = false}, false);
    robot::printPose();
    mogoMech.release();
    chassis.safeMoveToPoint(-22 * s, -19, 1000, {.forwards = false, .maxSpeed = 50}, false);
    robot::printPose();
    // pros::delay(100);
    mogoMech.clamp();
    chassis.safeMoveToPoint(-58 * s, -58, 1000, {.forwards = true, .maxSpeed = 70}, false);
    intake.setMode(Intake::modes::CONTINUOUS);
    intake.forwards();
    chassis.safeMoveToPoint(-57 * s, -57, 1000, {.forwards = true, .maxSpeed = 70});
    chassis.safeMoveToPoint(-12 * s, -30, 1000, {.forwards = true}, false);
    chassis.brake();
}
} // namespace auton