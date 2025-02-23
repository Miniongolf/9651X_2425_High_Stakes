#pragma once
#include "lemlib/pid.hpp"
#include "units/units.hpp" // IWYU pragma: keep

struct ColourRange {
        ColourRange(double min, double max)
            : min(min),
              max(max) {}

        double min, max;

        /**
         * @brief Check if the given hue is within the range
         *
         * @param true if the hue is within the range, false otherwise
         */
        [[nodiscard]] bool inRange(double value) const { return value >= min && value <= max; }
};

const ColourRange red(0, 30);
const ColourRange blue(160, 240);

enum class Alliance { RED, BLUE, NONE };

/**
 * @brief Check if the two alliances are opposite colours
 * 
 * @param lhs alliance 1
 * @param rhs alliance 2
 * @return bool
 */
bool inline isOpposite(const Alliance lhs, const Alliance rhs) {
    if (lhs == Alliance::NONE || rhs == Alliance::NONE) return false;
    return lhs != rhs;
}

extern lemlib::PID emptyLateralPID, emptyAngularPID;
extern lemlib::PID mogoLateralPID, mogoAngularPID;