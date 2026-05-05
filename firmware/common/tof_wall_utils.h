#ifndef COMMON_TOF_WALL_UTILS_H
#define COMMON_TOF_WALL_UTILS_H

#include <cstdint>

#include "common/utils.h"
#include "config/sensors.h"

namespace tof_wall
{
enum class SteeringSource : uint8_t
{
    None,
    Left,
    Right
};

struct WallState
{
    bool           left_wall        = false;
    bool           front_wall       = false;
    bool           right_wall       = false;
    bool           side_error_valid = false;
    bool           steering_allowed = false;
    bool           front_blocked    = false;
    SteeringSource source           = SteeringSource::None;
    float          left_error_mm    = 0.0f;
    float          right_error_mm   = 0.0f;
    float          side_error_mm    = 0.0f;
};

inline const char* sourceName(SteeringSource source)
{
    switch (source)
    {
        case SteeringSource::Left:
            return "LEFT";
        case SteeringSource::Right:
            return "RIGHT";
        case SteeringSource::None:
        default:
            return "NONE";
    }
}

inline bool validReading(float distance_mm)
{
    return distance_mm > 0.0f && distance_mm < TOF_OUT_OF_RANGE_MM;
}

inline bool wallLeft(float distance_mm)
{
    return validReading(distance_mm) && distance_mm < TOF_LEFT_WALL_THRESHOLD_MM;
}

inline bool wallFront(float distance_mm)
{
    return validReading(distance_mm) && distance_mm < TOF_FRONT_WALL_THRESHOLD_MM;
}

inline bool wallRight(float distance_mm)
{
    return validReading(distance_mm) && distance_mm < TOF_RIGHT_WALL_THRESHOLD_MM;
}

inline bool frontWallTooCloseForSteering(float distance_mm)
{
    return validReading(distance_mm) && distance_mm < TOF_FRONT_WALL_RELIABILITY_LIMIT_MM;
}

inline WallState evaluate(float left_mm, float front_mm, float right_mm)
{
    WallState state;
    state.left_wall     = wallLeft(left_mm);
    state.front_wall    = wallFront(front_mm);
    state.right_wall    = wallRight(right_mm);
    state.front_blocked = frontWallTooCloseForSteering(front_mm);

    // Jurababa's yaw/omega convention is positive left and negative right.
    // Therefore negative side error steers right, and positive steers left.
    if (state.left_wall)
        state.left_error_mm = left_mm - TOF_LEFT_CENTER_REFERENCE_MM;
    if (state.right_wall)
        state.right_error_mm = TOF_RIGHT_CENTER_REFERENCE_MM - right_mm;

    if (state.left_wall && state.right_wall)
    {
        const float left_closeness_mm  = TOF_LEFT_CENTER_REFERENCE_MM - left_mm;
        const float right_closeness_mm = TOF_RIGHT_CENTER_REFERENCE_MM - right_mm;
        if (left_closeness_mm > right_closeness_mm)
        {
            state.side_error_mm = state.left_error_mm;
            state.source        = SteeringSource::Left;
        }
        else
        {
            state.side_error_mm = state.right_error_mm;
            state.source        = SteeringSource::Right;
        }
        state.side_error_valid = true;
    }
    else if (state.left_wall)
    {
        state.side_error_mm    = state.left_error_mm;
        state.side_error_valid = true;
        state.source           = SteeringSource::Left;
    }
    else if (state.right_wall)
    {
        state.side_error_mm    = state.right_error_mm;
        state.side_error_valid = true;
        state.source           = SteeringSource::Right;
    }

    state.steering_allowed = state.side_error_valid && !state.front_blocked;
    return state;
}

inline float steeringAdjustmentDegps(float side_error_mm, float side_error_delta_mmps)
{
    const float adjustment_degps = TOF_STEERING_KP_DEGPS_PER_MM * side_error_mm +
                                   TOF_STEERING_KD_DEG_PER_MM * side_error_delta_mmps;
    return utils::clampAbs(adjustment_degps, TOF_STEERING_ADJUST_LIMIT_DEGPS);
}
} // namespace tof_wall

#endif
