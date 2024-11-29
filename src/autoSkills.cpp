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
        {{-48, 59}, 750, 1500}, // rings 1+2
        {{-60, 48}, 750, 750}, // ring 3
        {{-24, 49}, 750, 750}, // ring 4
        {  {2, 62}, 750, 750}, // ring 5
        {{-25, 24}, 750, 1500}, // ring 6
        {{-55, 55}, 750, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, 70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();

    // Position reset
    chassis.moveToPoint(-49, 49, 1000);
    chassis.turnToHeading(0, 750, {}, false);
    // robot::moveTimed(100, 0, 1000);
    // chassis.setPose(chassis.getPose().x, 62, 0);

    // Mogo 2
    chassis.moveToPoint(-48, -15, 1500, {.forwards=false});
    chassis.waitUntil(60);
    chassis.moveToPoint(-48, -24, 1000, {.forwards=false, .maxSpeed=70}, false);
    robot::clampMogo(true);
    intake.forwards();
    robot::pathInterp({
        {{-47, -60}, 1000, 1500}, // rings 1+2
        {{-60, -48}, 750, 750}, // ring 3
        {{-24, -48}, 750, 750}, // ring 4
        {  {2, -55}, 750, 750}, // ring 5
        {{-24, -24}, 750, 1500}, // ring 6
        {{-55, -55}, 750, 1500, {.forwards=false}}, // corner
    });
    chassis.turnToPoint(-70, -70, 500, {.forwards=false}, false);
    robot::moveTimed(-50, 0, 400);
    robot::releaseMogo();
    intake.stop();

    chassis.moveToPoint(-48, -48, 1000);
    chassis.turnToHeading(0, 750, {}, false);
    robot::moveTimed(-60, 0, 750);
    pros::delay(100);
    chassis.setPose(-48, -60, 0);

    arm.moveToAngle(armPositions::load);

    // Cross field
    robot::pathInterp({{{-48, -60}, 500, 1000}});
    chassis.turnToPoint(35, 35, 1000, {}, false);
    pros::delay(200);
    chassis.moveToPoint(35, 35, 2000);
    chassis.waitUntil(72);
    intake.forwards();
    chassis.waitUntilDone();
    pros::delay(100);
    intake.stop();

    // Mogo 3
    chassis.turnToPoint(48, 0, 500, {.forwards=false}, false);
    pros::delay(200);
    chassis.moveToPoint(48, 0, 1500, {.forwards=false}, false);
    robot::clampMogo(true);
    arm.moveToAngle(armPositions::standby);
    intake.forwards();

    robot::pathInterp({
        {{24, 48}, 750, 1500}, // rings 3+4
        {{60, 48}, 750, 1000}, // ring 5
        {{48, 60}, 750, 750}, // ring 6
        {{60, 60}, 750, 1500, {.forwards=false}}, // corner
    });
}