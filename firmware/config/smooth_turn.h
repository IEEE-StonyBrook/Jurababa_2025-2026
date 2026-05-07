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

constexpr float SENSING_POSITION_MM = CELL_SIZE_MM - 10.0f; // Osmium SENSING_POSITION = 170
constexpr float FRONT_REFERENCE_MM =
    23.0f; // Measured on Jurababa: front ToF reads ~16-26 mm at true cell centre after stopAtCentre
constexpr float FRONT_CORRECTION_TOLERANCE_MM   = 5.0f;  // UKMARS adjustPosition tolerance = 50 raw
constexpr float FRONT_CORRECTION_STEP_MM        = 25.0f; // UKMARS correction step = 10 mm
constexpr float FRONT_CORRECTION_SPEED_MMPS     = 100.0f;  // UKMARS adj_speed = 100 mm/s
constexpr float FRONT_CORRECTION_ACCEL_MMPS2    = 1000.0f; // UKMARS adj_accel = 1000 mm/s^2
constexpr float STOP_AT_CENTRE_FRONT_SPEED_MMPS = 30.0f; // UKMARS front-wall final_speed = 30 mm/s
constexpr uint32_t STOP_AT_CENTRE_FRONT_TIMEOUT_MS   = 500;  // UKMARS front-wall timeout = 500 ms
constexpr uint32_t STOP_AT_CENTRE_PROFILE_TIMEOUT_MS = 1000; // UKMARS no-wall timeout = 1000 ms

// Positive adjustment delays the sensor-triggered turn when adjacent walls
// make the front ToF read short. Start with zero until measured on Jurababa.
constexpr float EXTRA_WALL_ADJUST_MM = 0.0f; // Osmium EXTRA_WALL_ADJUST = 5/6 raw analog

// Smooth-turn tuning notes
// ------------------------
// Each row is the UKMARS SS90E/SS90 shape:
//   speed, entry offset, exit offset, angle, omega, alpha, front trigger
//
// The turn starts while the mouse is crossing the next cell boundary. If the
// front ToF reaches front_trigger_mm before the distance trigger, the turn
// begins early from sensor truth; otherwise it begins at:
//   FULL_CELL + HALF_CELL - entry_offset_mm
//
// Tune in this order:
//   1. Keep speed/omega/alpha conservative until in-place turns are repeatable.
//   2. Tune front_trigger_mm so the turn starts at the same physical point with
//      and without a front wall. If side walls make the front ToF read short,
//      raise EXTRA_WALL_ADJUST_MM.
//   3. Tune entry_offset_mm for the start of the arc. Too small starts late and
//      clips the inside wall; too large starts early and drifts wide.
//   4. Tune exit_offset_mm so the robot finishes the arc aimed down the next
//      cell and reaches SENSING_POSITION_MM without a lateral step.
//   5. Tune omega_degps/alpha_degps2 only after geometry is close. Higher omega
//      shortens the radius at a fixed speed; higher alpha makes the heading
//      transition sharper and can expose IMU/controller lag.
//   6. Tune left and right rows independently. Tire loading, motor asymmetry,
//      and ToF mounting can make SS90EL and SS90ER differ.
//
// Osmium reference from ukmars/mazerunner-core config-robot-osmium.h is kept
// beside each active value below. Use it as a shape reference, not a direct
// target: Osmium uses analog wall sensors and encoder yaw, while Jurababa uses
// ToF millimeters and IMU yaw.

constexpr SmoothTurnParameters SMOOTH_TURN_PARAMS[] = {
    // speed, entry, exit, angle, omega, alpha, front trigger
    // Osmium: {300, 70, 80, +/-90, 287, 2866, raw trigger 82/110}
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, 90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM}, // Osmium SS90EL
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, -90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM}, // Osmium SS90ER
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, 90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM}, // Osmium SS90L
    {ROBOT_SEARCH_TURN_SPEED_MMPS, 70.0f, 80.0f, -90.0f, ROBOT_SMOOTH_TURN_OMEGA_DEGPS,
     ROBOT_SMOOTH_TURN_ALPHA_DEGPS2, ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM}, // Osmium SS90R
};

constexpr int SMOOTH_TURN_PARAM_COUNT =
    static_cast<int>(sizeof(SMOOTH_TURN_PARAMS) / sizeof(SMOOTH_TURN_PARAMS[0]));

#endif // CONFIG_SMOOTH_TURN_H
