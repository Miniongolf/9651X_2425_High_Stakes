#include "autonFuncts.hpp"
#include "constants.hpp"
#include "helperFuncts.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "subsys/arm.hpp"

void auton::alternateAutoSkills() {
    chassis.setPose(-64, 0, -90);
    
    // Arm alliance stake
    arm.moveToAngle(armPositions::allianceStake);
    pros::delay(1000);
    arm.moveToAngle(armPositions::standby);
    pros::delay(300);

    // Mogo 1 (left top)
    chassis.swingToPoint(-48, 24, lemlib::DriveSide::RIGHT, 1000, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(-48, 24, 1000, {.forwards=false}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-24, 24}, 1000, 500}, // ring 1
        {{-24, 48}, 1000, 500}, // ring 2
        {  {0, 60}, 1000, 500}, // ring 3
        {{-48, 60}, 1000, 500}, // ring 4
        {{-48, 48}, 1000, 500}, // ring 5
        {{-60, 48}, 1000, 500}, // ring 6
        {{-60, 60}, 1000, 500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, 70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();

    // Mogo 2
    robot::pathInterp({
        {{-48, 48}, 1000, 500},
        {{-48, -24}, 1000, 500, {.forwards=false}},
    });
    
    robot::clampMogo(true);
    robot::pathInterp({
        {{-24, -24}, 1000, 500}, // ring 1
        {{-24, -48}, 1000, 500}, // ring 2
        {  {0, -60}, 1000, 500}, // ring 3
        {{-48, -60}, 1000, 500}, // ring 4
        {{-48, -48}, 1000, 500}, // ring 5
        {{-60, -48}, 1000, 500}, // ring 6
        {{-60, -60}, 1000, 500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, -70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
}