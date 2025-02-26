#pragma once

#include "math.h"
#include "lemlib/curve.hpp" // IWYU pragma: keep
#include "Eigen/Dense"
using namespace Eigen;

namespace catlib {
    double toRadian(double degree);

    double toDeg(double radian);

    double to0_360(double degree);

    double toNegPos180(double degree);

    double toNegPos90(double degree);

    double left_velocity_scaling(double drive_output, double heading_output);

    double right_velocity_scaling(double drive_output, double heading_output);

    bool is_line_settled(float targetX, float targetY, float desired_angle_deg, float currentX, float currentY);

    double limit(double input, double min, double max);

    double limit_min(double drive_output, double drive_min_voltage);

    double curvatureToAPoint(Vector2d currentPose, double currentHeading, Vector2d targetPose);
}