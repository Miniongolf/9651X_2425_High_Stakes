#pragma once

#include "robot/helperFuncts.hpp" // IWYU pragma: keep
#include "units/units.hpp"

namespace auton {
    extern int splitNum;
    extern Time startTime;
    void resetSplit();
    void nextSplit(std::string name);

    void twoRing(bool positive);
    void safeAWP(bool positive);

    void sigSAWP();

    void skills();
    void tunePID(bool hasMogo);
    void ringRush();
    void blueGoalRush();
    void newBlueGoalRush();
    void goalRush();
}