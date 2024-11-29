#include "autonFuncts.hpp"
#include "constants.hpp"
#include "helperFuncts.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "subsys/arm.hpp"

void auton::alternateAutoSkills() {
    chassis.setPose(-57, 0, -90);
    robot::moveTimed(50, 0, 500);
    
    // Arm alliance stake
    arm.moveToAngle(30);
    pros::delay(750);
    arm.moveToAngle(armPositions::standby);
    pros::delay(300);

    // Mogo 1 (left top)
    chassis.swingToPoint(-48, 24, lemlib::DriveSide::RIGHT, 1000, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(-48, 24, 1000, {.forwards=false}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-48, 70}, 500, 750}, // ring 5
        {{-60, 48}, 500, 750}, // ring 6
        {{-24, 48}, 500, 750}, // ring 2
        {  {0, 60}, 500, 750}, // ring 3
        {{-24, 24}, 500, 1500}, // ring 1
        {{-55, 55}, 500, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, 70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();

    // Position reset
    chassis.moveToPoint(-48, 48, 1000);
    chassis.turnToHeading(0, 500);
    robot::moveTimed(100, 0, 500);
    chassis.setPose(chassis.getPose().x, 62, 0);

    // Mogo 2
    chassis.moveToPoint(-48, -24, 1500, {.forwards=false}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-48, -70}, 500, 750}, // ring 5
        {{-60, -48}, 500, 750}, // ring 6
        {{-24, -48}, 500, 750}, // ring 2
        {  {0, -60}, 500, 750}, // ring 3
        {{-24, -24}, 500, 1500}, // ring 1
        {{-55, -55}, 500, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, -70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();
}