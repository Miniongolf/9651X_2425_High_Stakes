#pragma once

#include "robot/helperFuncts.hpp" // IWYU pragma: keep

namespace auton {
    void skills();
    void tunePID(bool hasMogo);
    void ringRush();
    void blueGoalRush();
    void goalRush();
}