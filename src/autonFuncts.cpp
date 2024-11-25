#include "autonFuncts.hpp"
#include "constants.hpp"
#include "helperFuncts.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "subsys/arm.hpp"

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

void testBoomerang() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(36, 36, 90, 5000, {.minSpeed=70});
    chassis.waitUntilDone();
    pros::delay(100);
    robot::printPose();
    pros::delay(100);
}

void soloAWP() {
    int s = isRedAlliance ? 1 : -1;
    chassis.setPose(-56*s, 11, 0);

    // Alliance stake
    robot::moveTimed(-75, 0, 500);
    chassis.moveToPoint(-60, 0, 750, {.forwards=false}, false);
    pros::delay(200);
    chassis.turnToHeading(-90, 750, {}, false);
    robot::moveTimed(50, 0, 400);
    arm.moveToAngle(30);
    pros::delay(1000);
    arm.moveToAngle(armPositions::standby);
    pros::delay(500);

    // Grab mogo
    chassis.turnToPoint(-24, 24, 750, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(-24, 24, 1000, {.forwards=false}, false);
    pros::delay(300);
    robot::clampMogo();
    pros::delay(500);

    // Mid stack
    intake.forwards();
    chassis.turnToPoint(-11, 41, 500);
    chassis.moveToPoint(-11, 41, 1000, {.maxSpeed=90}, false);
    chassis.moveToPoint(-11, 52, 600, {.maxSpeed=90}, false);
    pros::delay(2000);
    chassis.turnToPoint(-24, 48, 200);
    chassis.moveToPoint(-24, 48, 1500, {}, false);
    chassis.moveToPoint(-24, 15, 1500, {.maxSpeed=100}, false);
    robot::moveTimed(60, 0, 1000);
}

void safeSAWP() {
    int s = isRedAlliance ? 1 : -1;
    chassis.setPose(-60, 24, -90);

    // Mogo 1
    chassis.moveToPoint(-24, 24, 1500, {.forwards=false}, false);
    robot::clampMogo();
    pros::delay(500);
    
    // Ring 1
    intake.forwards();
    chassis.turnToPoint(-24, 48, 1000);
    pros::delay(300);
    chassis.moveToPoint(-24, 45, 1000);

    // // Cross
    // chassis.moveToPoint(-48, -12, 2000, {});
    // chassis.waitUntil(20);
    // robot::releaseMogo();
    // intake.stop();
    // pros::delay(500);
    // robot::releaseMogo();

    // // Mogo 2
    // chassis.moveToPoint(-20, -24, 2000, {.forwards=false}, false);
    // pros::delay(500);
    // robot::clampMogo();

    // // Ring 2
    // intake.forwards();
    // chassis.turnToPoint(-22*s, -45, 400);
    // pros::delay(1000);
    // chassis.moveToPoint(-22*s, -45, 1000, {}, false);

    // Touch bar
    chassis.moveToPoint(-15, 15, 1000, {.minSpeed=50});
    chassis.turnToHeading(135, 600, {}, false);
    robot::printPose();
    arm.moveToAngle(30);
    pros::delay(1000);
    robot::moveTimed(50, 0, 450);
}

void safeAWP() {
    chassis.setPose(-60, 24, -90);

    // Mogo 1
    chassis.moveToPoint(-24, 24, 1500, {.forwards=false}, false);
    robot::clampMogo();
    pros::delay(500);
    
    // Ring 1
    intake.forwards();
    chassis.turnToPoint(-24, 48, 1000);
    pros::delay(300);
    chassis.moveToPoint(-24, 45, 1000);
    
    // Touch bar
    chassis.moveToPoint(-15, 15, 1000);
    chassis.turnToPoint(0, 0, 1000);
    arm.moveToAngle(11);
}

void redMogoRush() {
    doinker.retract();
    chassis.setPose(-58, -60, 75);

    // Mogo rush
    intake.forwards();
    chassis.moveToPoint(-24, -48, 850, {.minSpeed=80}, false);
    robot::printPose();
    chassis.turnToPoint(-4, -42, 300, {}, false);
    chassis.waitUntil(30);
    doinker.extend();
    chassis.waitUntilDone();
    pros::delay(100);
    intake.stop();
    chassis.arcade(-80, 0);
    pros::delay(700);
    chassis.arcade(0, 0);
    doinker.retract();
    pros::delay(500);

    // Clamp rushed mogo
    chassis.turnToPoint(-8, -55, 750, {.forwards=false}, false);
    chassis.moveToPoint(-8, -55, 1000, {.forwards=false}, false);
    robot::clampMogo();
    pros::delay(100);

    // 2nd ring
    intake.forwards();
    chassis.moveToPoint(-35, -57, 2000, {.forwards=true}, false);
    pros::delay(1000);
    chassis.moveToPoint(-50, -61, 2000, {.forwards=true, .maxSpeed=80}, false);
    robot::printPose();
    chassis.turnToHeading(90, 750, {AngularDirection::CCW_COUNTERCLOCKWISE});
    pros::delay(300);
    intake.stop();
    robot::releaseMogo();
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, chassis.getPose().y, 1000);

    // Clamp second mogo
    chassis.moveToPoint(-24, chassis.getPose().y, 1000, {}, false);
    pros::delay(300);
    chassis.turnToPoint(-20, -24, 750, {.forwards=false}, false);
    chassis.moveToPoint(-20, -24, 1500, {.forwards=false}, false);
    pros::delay(200);
    chassis.waitUntilDone();
    robot::printPose();

    robot::clampMogo();
    pros::delay(500);
    intake.forwards();
    pros::delay(250);
    chassis.moveToPoint(chassis.getPose().x, -60, 1000, {}, false);
    pros::delay(250);
    chassis.turnToPoint(70, -70, 1000);
    chassis.waitUntilDone();
}

void blueMogoRush() {
    doinker.retract();
    chassis.setPose(56, -59, -90);

    // Mogo rush
    chassis.moveToPoint(21, -59, 900, {.minSpeed=80}, false);
    robot::printPose();
    chassis.turnToPoint(0, -51, 1000, {}, false);
    // chassis.waitUntil(30);
    chassis.waitUntilDone();
    doinker.extend();
    pros::delay(100);
    chassis.arcade(-80, 0);
    pros::delay(400);
    pros::delay(300);
    chassis.arcade(0, 0);
    doinker.retract();
    pros::delay(500);

    // // Clamp rushed mogo
    chassis.turnToPoint(15, -60, 750, {.forwards=false}, false);
    chassis.moveToPoint(15, -60, 750, {.forwards=false}, false);
    robot::clampMogo();
    pros::delay(100);
    intake.forwards();

    chassis.moveToPoint(24, -48, 2000, {.forwards=true}, false);
    pros::delay(1000);
    chassis.moveToPoint(24, -48, 2000, {.forwards=true, .maxSpeed=80}, false);
    pros::delay(500);
    chassis.turnToPoint(-70, -70, 1000);
    // robot::printPose();
    // chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 750);
    // robot::releaseMogo();
    // intake.stop();
    // chassis.moveToPoint(24, chassis.getPose().y, 1000, {}, false);
    // chassis.moveToPoint(24, -26, 1000, {.forwards=false});
    // pros::delay(200);
    // chassis.waitUntilDone();
    // robot::printPose();

    // robot::clampMogo();
    // pros::delay(500);
    // intake.forwards();
    // pros::delay(250);
    // chassis.turnToPoint(-3, 15, 500);
    // chassis.moveToPoint(5, -7, 750);
    // chassis.waitUntilDone();
    // doinker.extend();
    // robot::moveTimed(-80, 0, 500);
    // doinker.retract();
    // chassis.moveToPoint(24, -24, 1000);
    // chassis.waitUntilDone();
}

void autoSkills() {
    chassis.setPose(-56, 0, -90);
    // Alliance stake
    arm.moveToAngle(armPositions::allianceStake);
    pros::delay(1000);
    lemlib::Timer moveBackTimer(2000);
    chassis.arcade(-127, 0);
    while (chassis.getPose().y < -43 && !moveBackTimer.isDone()) {
        pros::delay(10);
    }
    chassis.arcade(0, 0);

    // Grab mogo
    chassis.turnToPoint(-48, 24, 1000, {.forwards=false}, false);
    pros::delay(300);
    chassis.moveToPoint(-48, 24, 2000, {.forwards=false}, false);
    robot::clampMogo();
    chassis.turnToPoint(-60, 48, 1000);

    // Ring 1
    intake.forwards();
    chassis.moveToPoint(-60, 48, 2000, {}, false);
    pros::delay(500);

    // Ring 2
    chassis.moveToPoint(-48, 60, 2000);

    // Corner 1
    chassis.turnToPoint(-65, 65, 2000, {.forwards=false}, false);
    pros::delay(500);
    robot::moveTimed(-60, 0, 500);

    // Ring 3
    chassis.turnToPoint(-48, 48, 1000);
    chassis.moveToPoint(-48, 48, 2000, {}, false);
    pros::delay(500);
    intake.stop();

    // Mogo 2
    chassis.turnToPoint(-24, -48, 750, {.forwards=false});
    chassis.moveToPoint(-24, -48, 2000, {.forwards=false});
    pros::delay(300);
    robot::clampMogo();
    pros::delay(300);

    // Ring 4
    intake.forwards();
    chassis.turnToPoint(-60, -48, 750);
    chassis.moveToPoint(-60, -48, 2000, {}, false);
    pros::delay(500);
    chassis.turnToPoint(-65, -65, 750, {.forwards=false});
    chassis.moveToPoint(-65, -65, 2000, {.forwards=false});
    robot::releaseMogo();
}
} // namespace auton