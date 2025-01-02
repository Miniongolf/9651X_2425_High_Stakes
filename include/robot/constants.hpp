#pragma once

namespace robot {
    constexpr double intakeOffset = 8.0;
    constexpr double mogoOffset = -5.0;
    constexpr double wallStakeOffset = 11.0;
    constexpr double allianceStakeOffset = 13.0;
}

struct ColourRange {
    ColourRange(int min, int max) : min(min), max(max) {}
    int min, max;
    
    /**
     * @brief Check if the given hue is within the range
     * 
     * @param true if the hue is within the range, false otherwise
     */
    [[nodiscard]] bool inRange(int value) const {
        return value >= min && value <= max;
    }
};

const ColourRange red(0, 25);
const ColourRange blue(150, 250);