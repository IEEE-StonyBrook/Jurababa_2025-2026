#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <cmath>

/**
 * @brief Enum for specifying left or right wheel/motor
 *
 * Replaces error-prone string-based dispatch ("left"/"right")
 * with type-safe enum for better performance and compile-time checking.
 */
enum class WheelSide
{
    LEFT,
    RIGHT
};

/**
 * @brief Namespace for robot utility functions
 *
 * Contains math utilities, angle wrapping, and common calculations
 * used across the robot control system.
 */
namespace utils
{
    /**
     * @brief Wraps angle to [-180, 180] degree range
     *
     * @param degrees Angle in degrees (any range)
     * @return Wrapped angle in [-180, 180] degrees
     */
    inline float wrapAngle180(float degrees)
    {
        degrees = std::fmod(degrees + 180.0f, 360.0f);
        if (degrees < 0.0f)
            degrees += 360.0f;
        return degrees - 180.0f;
    }

    /**
     * @brief Snaps angle to nearest 45-degree increment
     *
     * @param degrees Angle in degrees
     * @return Snapped angle (0, +/-45, +/-90, +/-135, 180)
     */
    inline float snapTo45(float degrees)
    {
        float snapped = 45.0f * std::round(degrees / 45.0f);
        snapped = wrapAngle180(snapped);
        if (snapped == -180.0f)
            snapped = 180.0f;
        return snapped;
    }

    /**
     * @brief Clamps value to symmetric range [-maxAbs, +maxAbs]
     */
    inline float clampAbs(float value, float maxAbs)
    {
        if (value > maxAbs)
            return maxAbs;
        if (value < -maxAbs)
            return -maxAbs;
        return value;
    }

    /**
     * @brief Calculates stopping distance for given velocity and deceleration
     *
     * Uses kinematic equation: s = (v^2 - u^2) / (2a)
     */
    inline float stoppingDistance(float initialVel, float finalVel, float decel)
    {
        float vInitSq = initialVel * initialVel;
        float vFinalSq = finalVel * finalVel;
        return (vInitSq - vFinalSq) / (2.0f * decel);
    }

    /**
     * @brief Calculates maximum safe velocity to reach target from current distance
     *
     * Uses kinematic equation: v = sqrt(u^2 + 2as)
     */
    inline float maxVelocityForDistance(float distance, float decel, float finalVel = 0.0f)
    {
        float vFinalSq = finalVel * finalVel;
        float vMaxSq = vFinalSq + 2.0f * decel * distance;
        return std::sqrt(vMaxSq);
    }
}

#endif
