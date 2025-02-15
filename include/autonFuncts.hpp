#pragma once

#include "robot/helperFuncts.hpp" // IWYU pragma: keep

namespace auton {
void skills();
void tunePID(bool hasMogo);
void safeShort();
void soloAWP();
void twoMogoSolo();
void elims();
}