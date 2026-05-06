/**
 * @file smooth_turn.h
 * @brief UKMARS-style smooth turn configuration.
 */
#ifndef CONFIG_SMOOTH_TURN_H
#define CONFIG_SMOOTH_TURN_H

#include <cstdint>

#include "config/geometry.h"
#include "config/motion.h"

struct SmoothTurnParameters
{
    float speed_mmps;
    float entry_offset_mm;
    float exit_offset_mm;
    float angle_deg;
    float omega_degps;
    float alpha_degps2;
    float front_trigger_mm;
};

enum SmoothTurnId
{
    SS90EL = 0,
    SS90ER = 1,
    SS90L  = 2,
    SS90R  = 3,
};

constexpr float    SENSING_POSITION_MM               = CELL_SIZE_MM - 10.0f;
constexpr float    FRONT_REFERENCE_MM                = 27.0f;
constexpr float    FRONT_CORRECTION_TOLERANCE_MM     = 5.0f;
constexpr float    FRONT_CORRECTION_STEP_MM          = 10.0f;
constexpr float    FRONT_CORRECTION_SPEED_MMPS       = 100.0f;
constexpr float    FRONT_CORRECTION_ACCEL_MMPS2      = 1000.0f;
constexpr float    STOP_AT_CENTRE_FRONT_SPEED_MMPS   = 30.0f;
constexpr uint32_t STOP_AT_CENTRE_FRONT_TIMEOUT_MS   = 500;
constexpr uint32_t STOP_AT_CENTRE_PROFILE_TIMEOUT_MS = 1000;

// Positive adjustment delays the sensor-triggered turn when adjacent walls
// make the front ToF read short. Start with zero until measured on Jurababa.
constexpr float EXTRA_WALL_ADJUST_MM = 0.0f;

constexpr SmoothTurnParameters SMOOTH_TURN_PARAMS[] = {
    // speed, entry, exit, angle, omega, alpha, front trigger
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, 90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM},
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, -90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM},
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, 90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM},
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, -90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM},
};

constexpr int SMOOTH_TURN_PARAM_COUNT =
    static_cast<int>(sizeof(SMOOTH_TURN_PARAMS) / sizeof(SMOOTH_TURN_PARAMS[0]));

#endif // CONFIG_SMOOTH_TURN_H
