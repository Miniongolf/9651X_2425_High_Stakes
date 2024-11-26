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
    chassis.setPose(-59*s, 11, 0);

    // Alliance stake
    robot::moveTimed(-75, 0, 500);
    chassis.moveToPoint(-60*s, 0, 750, {.forwards=false}, false);
    pros::delay(200);
    chassis.turnToHeading(-90*s, 750, {}, false);
    robot::moveTimed(50, 0, 400);
    arm.moveToAngle(30);
    pros::delay(1000);
    arm.moveToAngle(armPositions::standby);
    pros::delay(200);

    // Grab mogo
    chassis.turnToPoint(-24*s, 24, 750, {.forwards=false}, false);
    pros::delay(100);
    chassis.moveToPoint(-23*s, 24, 1500, {.forwards=false, .maxSpeed=100}, false);
    pros::delay(150);
    robot::clampMogo();
    pros::delay(200);

    // Mid stack
    intake.forwards();
    chassis.turnToPoint(-13*s, 38, 500);
    chassis.moveToPoint(-13*s, 38, 1000, {.maxSpeed=90}, false);
    chassis.moveToPoint(-12*s, 52, 600, {.maxSpeed=90}, false);
    pros::delay(1000);
    chassis.turnToPoint(-26*s, 47, 200);
    chassis.moveToPoint(-26*s, 47, 1500, {}, false);
    pros::delay(300);

    // Touch bar
    chassis.moveToPoint(-24*s, 15, 1500, {.maxSpeed=100}, false);
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
    pros::delay(400);

    // Clamp rushed mogo
    chassis.turnToPoint(-12, -55, 750, {.forwards=false}, false);
    chassis.moveToPoint(-12, -55, 1000, {.forwards=false}, false);
    robot::clampMogo();
    // pros::delay(60);

    // 2nd ring
    intake.forwards();
    chassis.moveToPoint(-35, -57, 1900, {.forwards=true}, false);
    // pros::delay(5);
    chassis.moveToPoint(-56, -61, 1400, {.forwards=true, .maxSpeed=80}, false);
    robot::printPose();
    chassis.turnToHeading(90, 750, {AngularDirection::CCW_COUNTERCLOCKWISE});
    pros::delay(80);
    intake.stop();
    pros::delay(200);
    robot::releaseMogo();
    // chassis.waitUntilDone();
    chassis.moveToPoint(-24, chassis.getPose().y, 300);

    // Clamp second mogo
    chassis.moveToPoint(-26, chassis.getPose().y, 1000, {}, false);
    pros::delay(5);
    // chassis.turnToPoint(-20, -24, 750, {.forwards=false}, false);
    // chassis.moveToPoint(-20, -24, 1500, {.forwards=false}, false);
     chassis.turnToPoint(-22, -24, 650, {.forwards=false}, false);
    chassis.moveToPoint(-22, -24, 1200, {.forwards=false}, false);
    pros::delay(5);
    chassis.waitUntilDone();
    robot::printPose();

    robot::clampMogo();
    pros::delay(20);
    intake.forwards();
    pros::delay(100);
    // chassis.moveToPoint(chassis.getPose().x, -60, 1000, {}, false);
    // pros::delay(50);
    // chassis.turnToPoint(70, -70, 1000);
    chassis.moveToPoint(-46, -53, 1000, {}, false);
    chassis.waitUntilDone();
}

void blueMogoRush() {
    doinker.retract();
    chassis.setPose(56, -59, -90);

    // Mogo rush
    chassis.moveToPoint(18, -59, 900, {.minSpeed=80}, false);
    robot::printPose();
    chassis.turnToPoint(0, -51, 400, {}, false);
    // chassis.waitUntil(30);
    // chassis.waitUntilDone();
    doinker.extend();
    pros::delay(50);
    chassis.arcade(-80, 0);
    pros::delay(400);
    pros::delay(300);
    chassis.arcade(0, 0);
    doinker.retract();
    pros::delay(500);

    // // Clamp rushed mogo
    chassis.turnToPoint(8, -55, 750, {.forwards=false}, false);
    chassis.moveToPoint(8, -55, 750, {.forwards=false}, false);
    pros::delay(200);
    robot::clampMogo();
    pros::delay(100);
    intake.forwards();
    pros::delay(400);
    chassis.moveToPoint(24, -48, 2000, {.forwards=true}, false);
    pros::delay(100);
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
    chassis.setPose(-64, 0, 0);
    // Alliance stake
    arm.moveToAngle(armPositions::allianceStake);
    pros::delay(1000);
    robot::moveTimed(-80, 0, 750);
    arm.moveToAngle(armPositions::standby);
    chassis.turnToPoint(-47.268, 23.67, 2000, {}, false);
    chassis.moveToPoint(-47.268, 23.67, 2000, {}, false);
    chassis.turnToPoint(-66.31, 66.31, 2000, {}, false);
    chassis.moveToPoint(-66.31, 66.31, 2000, {}, false);
    
    
    //Shove Mogo in corner
    
    // chassis.setPose(-54, 0, 0);
    // // Alliance stake
    // arm.moveToAngle(armPositions::allianceStake);
    // pros::delay(1000);
    // arm.moveToAngle(armPositions::standby);
    // pros::delay(300);
    // chassis.moveToPoint(0,44, 1000,{.forwards=false});
    // chassis.turnToPoint(-47.169,23.623, 1000);

    // // Grab mogo
    // chassis.moveToPoint(-47.169,23.623, 1000);
    // pros::delay(300);
    // robot::clampMogo();
    // chassis.turnToPoint(-23.383, 23.683, 1000);

    // // Ring 1
    // intake.forwards();
    // chassis.moveToPoint(-23.383, 23.683, 2000, {}, false);
    // pros::delay(200);

    // // Ring 2
    // chassis.turnToPoint(-23.622, 47.244, 1000);
    // chassis.moveToPoint(-23.622, 47.244, 2000);
    // pros::delay(200);

    // // Ring 3

    // chassis.turnToPoint(0, 59.055, 1000);
    // chassis.moveToPoint(0, 59.055, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();

    // // Ring 4
    // chassis.turnToPoint(-47.244, 47.244, 1000);
    // chassis.moveToPoint(-47.244, 47.244, 2000);
    // pros::delay(200);

    // // Ring 5
    // chassis.turnToPoint(-59.063, 47.244, 1000);
    // chassis.moveToPoint(-59.063, 47.244, 2000);
 
    // pros::delay(200);

    // // Ring 6
    // chassis.turnToPoint(-47.244, 59.055, 1000);
    // chassis.moveToPoint(-47.244, 59.055, 2000);
    // pros::delay(200);

    // // Corner 1
    // chassis.turnToPoint(-66.929, 66.929, 2000, {.forwards=false}, false);
    // pros::delay(200);
    // robot::moveTimed(-60, 0, 500);
    // robot::releaseMogo();

    // // Ring 7

    // chassis.turnToPoint(23.622, 47.244, 1000);
    // intake.forwards();
    // chassis.moveToPoint(23.622, 47.244, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();

    // // Ring 8

    // chassis.turnToPoint(23.71, 23.503, 1000);
    // intake.forwards();
    // chassis.moveToPoint(23.71, 23.503, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();

    // // Mogo 2
    // chassis.turnToPoint(47.496, 0, 750, {.forwards=false});
    // chassis.moveToPoint(47.496, 0, 2000, {.forwards=false});
    // pros::delay(300);
    // robot::clampMogo();
    // pros::delay(300);

    // // Ring 9

    // chassis.turnToPoint(23.868, -23.688, 1000);
    // intake.forwards();
    // chassis.moveToPoint(23.868, -23.688, 2000, {}, false);
    // pros::delay(200);

    // // Ring 10

    // chassis.turnToPoint(23.591, -47.268, 1000);
    // chassis.moveToPoint(23.591, -47.268, 2000, {}, false);
    // pros::delay(200);
    
    // // Ring 11

    // chassis.turnToPoint(0, -58.909, 1000);
    // chassis.moveToPoint(0, -58.909, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();

    // // Corner 2
    // chassis.turnToPoint(67, -67, 2000, {.forwards=false}, false);
    // pros::delay(200);
    // robot::moveTimed(-60, 0, 500);
    // robot::releaseMogo();

    // // Ring 11

    // chassis.turnToPoint(0, 0, 1000);
    // intake.forwards();
    // chassis.moveToPoint(0, 0, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();
    // // Ring 12

    // chassis.turnToPoint(-23.743, -23.83, 1000);
    // intake.forwards();
    // chassis.moveToPoint(-23.743, -23.83, 2000, {}, false);
    // pros::delay(200);
    // intake.stop();

    // // Mogo 3
    // chassis.turnToPoint(-47.049, -23.349, 750, {.forwards=false});
    // chassis.moveToPoint(-47.049, -23.349, 2000, {.forwards=false});
    // pros::delay(300);
    // robot::clampMogo();
    // pros::delay(300);
    // intake.forwards();

    // // Ring 13

    // chassis.turnToPoint(23.503, -47.136, 1000);
    // intake.forwards();
    // chassis.moveToPoint(23.503, -47.136, 2000, {}, false);
    // pros::delay(200);

    // // Ring 14

    // chassis.turnToPoint(-47.049, -47.136, 1000);
    // chassis.moveToPoint(-47.049, -47.136, 2000, {}, false);
    // pros::delay(200);

    // // Ring 15

    // chassis.turnToPoint(-47.29, -58.909, 1000);
    // chassis.moveToPoint(-47.29, -58.909, 2000, {}, false);
    // pros::delay(200);

    // // Ring 16
    // chassis.turnToPoint(-58.822, -47.136, 1000);
    // chassis.moveToPoint(-58.822, -47.136, 2000, {}, false);
    // pros::delay(200);

    // // Corner 3
    // chassis.turnToPoint(-66, -66, 2000, {.forwards=false}, false);
    // pros::delay(200);
    // robot::moveTimed(-60, 0, 500);
    // intake.stop();
}

// void BruteForceAutoSkills(){
//     chassis.setPose(-54, 0, 0);
//     // Alliance stake
//     arm.moveToAngle(armPositions::allianceStake);
//     pros::delay(1000);
//     robot::moveTimed(-80, 0, 700);
//     arm.moveToAngle(armPositions::standby);
//     //Shove Mogo in corner
//     chassis.turnToHeading(-135, 600, {}, false);
//     robot::moveTimed(-80, 0, 10000);
// }
} // namespace auton



