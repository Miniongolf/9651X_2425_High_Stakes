#include "robot/chassis/chassis.hpp"

void CustomChassis::moveTimed(float throttle, float steering, int time, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveTimed(throttle, steering, time, false); });
        pros::delay(10); // delay to give the task time to start
        return;
    }

    this->arcade(throttle, steering, true);
    pros::delay(time);
    this->arcade(0, 0);
};