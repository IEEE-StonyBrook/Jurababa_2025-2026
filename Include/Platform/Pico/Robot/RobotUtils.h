#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

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
namespace RobotUtils
{
    /**
     * @brief Wraps angle to [-180, 180] degree range
     *
     * Ensures angle is in standard range for yaw calculations.
     * Uses efficient modulo approach instead of loops.
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
     * Used for aligning robot to cardinal/diagonal directions
     * in the micromouse maze grid.
     *
     * @param degrees Angle in degrees
     * @return Snapped angle (0, ±45, ±90, ±135, 180)
     */
    inline float snapTo45Degrees(float degrees)
    {
        float snapped = 45.0f * std::round(degrees / 45.0f);
        snapped = wrapAngle180(snapped);

        // Normalize -180 to 180 (both represent same direction)
        if (snapped == -180.0f)
            snapped = 180.0f;

        return snapped;
    }

    /**
     * @brief Clamps value to symmetric range [-maxAbs, +maxAbs]
     *
     * @param value Input value
     * @param maxAbs Maximum absolute value
     * @return Clamped value
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
     * Uses kinematic equation: v² = u² + 2as
     * Solving for s: s = (v² - u²) / (2a)
     *
     * @param initialVelocityMMps Starting velocity (mm/s)
     * @param finalVelocityMMps Ending velocity (mm/s)
     * @param decelerationMMps2 Deceleration rate (mm/s²), positive value
     * @return Stopping distance in mm
     */
    inline float calculateStoppingDistance(float initialVelocityMMps,
                                          float finalVelocityMMps,
                                          float decelerationMMps2)
    {
        float vInitSq = initialVelocityMMps * initialVelocityMMps;
        float vFinalSq = finalVelocityMMps * finalVelocityMMps;
        return (vInitSq - vFinalSq) / (2.0f * decelerationMMps2);
    }

    /**
     * @brief Calculates maximum safe velocity to reach target from current distance
     *
     * Uses kinematic equation: v = √(u² + 2as)
     * For deceleration to rest: v = √(2 * a * s)
     *
     * @param distanceToTargetMM Distance remaining (mm)
     * @param decelerationMMps2 Deceleration rate (mm/s²), positive value
     * @param finalVelocityMMps Desired final velocity at target (mm/s)
     * @return Maximum velocity to reach target safely (mm/s)
     */
    inline float calculateMaxVelocityForDistance(float distanceToTargetMM,
                                                 float decelerationMMps2,
                                                 float finalVelocityMMps = 0.0f)
    {
        float vFinalSq = finalVelocityMMps * finalVelocityMMps;
        float vMaxSq = vFinalSq + 2.0f * decelerationMMps2 * distanceToTargetMM;
        return std::sqrt(vMaxSq);
    }
}

#endif
