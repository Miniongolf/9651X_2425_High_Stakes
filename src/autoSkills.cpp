#include "autonFuncts.hpp"
#include "constants.hpp"
#include "helperFuncts.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "subsys/arm.hpp"

void auton::alternateAutoSkills() {
    // Start position
    chassis.setPose(-57, 0, -90);
    
    // Arm alliance stake
    robot::moveTimed(50, 0, 500);
    arm.moveToAngle(30);
    pros::delay(750);
    arm.moveToAngle(armPositions::standby);
    pros::delay(100);

    // Mogo 1 (left top)
    chassis.swingToPoint(-48, 24, lemlib::DriveSide::RIGHT, 1000, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(-48, 24, 1000, {.forwards=false}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-48, 70}, 500, 750}, // rings 1+2
        {{-60, 48}, 500, 750}, // ring 3
        {{-24, 48}, 500, 750}, // ring 4
        {  {0, 60}, 500, 750}, // ring 5
        {{-24, 24}, 500, 1500}, // ring 6
        {{-55, 55}, 500, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, 70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();

    // Position reset
    chassis.moveToPoint(-48, 48, 1000);
    chassis.turnToHeading(0, 750, {}, false);
    robot::moveTimed(100, 0, 1000);
    chassis.setPose(chassis.getPose().x, 62, 0);

    // Mogo 2
    chassis.moveToPoint(-48, -24, 1500, {.forwards=false}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-48, -70}, 500, 750}, // rings 1+2
        {{-60, -48}, 500, 750}, // ring 3
        {{-24, -48}, 500, 750}, // ring 4
        {  {0, -60}, 500, 750}, // ring 5
        {{-24, -24}, 500, 1500}, // ring 6
        {{-55, -55}, 500, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, -70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();

    // Cross field
    arm.moveToAngle(armPositions::load);
    chassis.moveToPoint(24, 48, 2000);
    chassis.waitUntil(80);
    intake.forwards();
    chassis.waitUntilDone();
    intake.stop();

    // Mogo 3
    chassis.turnToPoint(48, 0, 500, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(48, 0, 1500, {.forwards=false}, false);
    robot::clampMogo(true);
    arm.moveToAngle(armPositions::standby);
    intake.forwards();
}