#pragma once

class ColourRange {
    public:
        ColourRange(double min, double max) : min(min), max(max) {};
        [[nodiscard]] bool inRange(double val) const { return val >= this->min && val <= this->max; }
    private:
        double min, max;
};
