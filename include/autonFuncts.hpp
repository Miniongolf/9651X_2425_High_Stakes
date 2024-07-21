#include "globals.hpp"

#pragma once

namespace auton {
void chassisGrabRing(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = false);
void chassisGrabMogo(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = false);

void leftWP();
void leftMax();
void rightWP();
void rightRush();
void skills();
}