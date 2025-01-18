#pragma once
#include "lemlib/pid.hpp"

namespace robot {
constexpr double intakeOffset = 8.0;
constexpr double mogoOffset = -5.0;
constexpr double wallStakeOffset = 11.0;
constexpr double allianceStakeOffset = 13.0;
} // namespace robot

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

const ColourRange red(0, 25);
const ColourRange blue(150, 250);

enum class Alliance { RED, BLUE, NONE };

/**
 * @brief Check if the two alliances are opposite colours
 * 
 * @param lhs alliance 1
 * @param rhs alliance 2
 * @return bool
 */
bool inline isOpposite(const Alliance lhs, const Alliance rhs) {
    return lhs == Alliance::RED ? rhs == Alliance::BLUE : rhs == Alliance::RED;
}

extern lemlib::PID emptyLateralPID, emptyAngularPID;
extern lemlib::PID mogoLateralPID, mogoAngularPID;