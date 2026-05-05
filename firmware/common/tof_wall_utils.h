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

// Mirrors UKMARS mazerunner-core's wall-detection + cross-track error
// shape (sensors.h:196-258 in /tmp/mazerunner-core), translated from
// IR-normalized counts to ToF mm via compile-time scale factors. The
// per-side scale absorbs per-unit chip bias, mounting, and cover variance:
// each side reads ~TOF_SIDE_NOMINAL when the robot is centered between
// two walls.
//
// Sign convention: positive side_error_norm means the robot has drifted
// toward the right wall, which commands a positive omega (CCW = left)
// correction via the rotation PD.
struct WallState
{
    bool           left_wall        = false;
    bool           front_wall       = false;
    bool           right_wall       = false;
    bool           side_error_valid = false;
    bool           steering_allowed = false;
    bool           front_blocked    = false;
    SteeringSource source           = SteeringSource::None;

    // All errors are in TOF_SIDE_NOMINAL units (100 = centered). Diagnostic
    // fields kept for both sides so logs can show which way the robot
    // actually drifted, even when the picker chose only one side.
    float left_error_norm  = 0.0f;
    float right_error_norm = 0.0f;
    float side_error_norm  = 0.0f;
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

    // Normalize each side's mm reading to TOF_SIDE_NOMINAL units so both
    // sides operate on the same scale (mazerunner config-robot-orion.h
    // LEFT_SCALE / RIGHT_SCALE pattern). After this multiply, both sides
    // produce ~TOF_SIDE_NOMINAL when the robot is centered, regardless of
    // their raw mm bias.
    const float lss_norm = TOF_LEFT_SCALE * left_mm;
    const float rss_norm = TOF_RIGHT_SCALE * right_mm;

    if (state.left_wall)
        state.left_error_norm = lss_norm - TOF_SIDE_NOMINAL;
    if (state.right_wall)
        state.right_error_norm = TOF_SIDE_NOMINAL - rss_norm;

    // Picker form lifted directly from mazerunner sensors.h:236-247. Closer
    // wall wins, with the same 2× factor applied in single-wall and
    // both-wall cases so the effective steering gain doesn't halve when
    // one wall ends — see audit item #5.
    if (state.left_wall && state.right_wall)
    {
        // Closer wall = smaller normalized reading (smaller mm × scale).
        if (lss_norm < rss_norm)
        {
            state.side_error_norm = 2.0f * state.left_error_norm;
            state.source          = SteeringSource::Left;
        }
        else
        {
            state.side_error_norm = 2.0f * state.right_error_norm;
            state.source          = SteeringSource::Right;
        }
        state.side_error_valid = true;
    }
    else if (state.left_wall)
    {
        state.side_error_norm  = 2.0f * state.left_error_norm;
        state.side_error_valid = true;
        state.source           = SteeringSource::Left;
    }
    else if (state.right_wall)
    {
        state.side_error_norm  = 2.0f * state.right_error_norm;
        state.side_error_valid = true;
        state.source           = SteeringSource::Right;
    }

    state.steering_allowed = state.side_error_valid && !state.front_blocked;
    return state;
}

inline float steeringAdjustmentDegps(float side_error_norm, float side_error_delta_norm_per_s)
{
    const float adjustment_degps = TOF_STEERING_KP_DEGPS_PER_NOMINAL * side_error_norm +
                                   TOF_STEERING_KD_DEG_PER_NOMINAL * side_error_delta_norm_per_s;
    return utils::clampAbs(adjustment_degps, TOF_STEERING_ADJUST_LIMIT_DEGPS);
}
} // namespace tof_wall

#endif
