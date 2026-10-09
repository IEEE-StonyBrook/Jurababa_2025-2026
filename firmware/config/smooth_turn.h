/**
 * @file smooth_turn.h
 * @brief UKMARS-style smooth turn configuration.
 */
#ifndef CONFIG_SMOOTH_TURN_H
#define CONFIG_SMOOTH_TURN_H

#include <cstdint>

#include "config/geometry.h"
#include "config/motion.h"

// Smooth turn speed/omega/alpha and the entry/exit corrections are runtime
// tuneable over the CLI, so they live in Motion::SmoothTurnTuning (seeded from
// config/motion.h) rather than a constexpr table here. The phase lengths are
// derived in control/smooth_turn_geometry.h.

constexpr float SENSING_POSITION_MM = CELL_SIZE_MM - 10.0f; // Osmium SENSING_POSITION = 170
constexpr float FRONT_REFERENCE_MM =
    45.0f; // Measured on Jurababa: front ToF reads ~16-26 mm at true cell centre after stopAtCentre
constexpr float FRONT_CORRECTION_TOLERANCE_MM   = 15.0f; // UKMARS adjustPosition tolerance = 50 raw
constexpr float FRONT_CORRECTION_STEP_MM        = 20.0f; // UKMARS correction step = 10 mm
constexpr float FRONT_CORRECTION_SPEED_MMPS     = 100.0f;  // UKMARS adj_speed = 100 mm/s
constexpr float FRONT_CORRECTION_ACCEL_MMPS2    = 1000.0f; // UKMARS adj_accel = 1000 mm/s^2
constexpr float STOP_AT_CENTRE_FRONT_SPEED_MMPS = 30.0f; // UKMARS front-wall final_speed = 30 mm/s
constexpr uint32_t STOP_AT_CENTRE_FRONT_TIMEOUT_MS   = 500;  // UKMARS front-wall timeout = 500 ms
constexpr uint32_t STOP_AT_CENTRE_PROFILE_TIMEOUT_MS = 1000; // UKMARS no-wall timeout = 1000 ms

// Smooth-turn tuning notes
// ------------------------
// Live knobs are edited over serial with `SMOOTH SET <name> <value>` and shown
// by `SMOOTH`: SPEED, OMEGA, ALPHA, and the per-side ENTRY_L/EXIT_L/ENTRY_R/
// EXIT_R corrections. Defaults come from config/motion.h; `SMOOTH DEFAULTS`
// restores them. Entry and exit are DERIVED from speed and omega
// (HALF_CELL - radius); the offsets are corrections on top, default 0.
//
// Tune in this order, using `SMOOTH RUN L|R` to measure each change:
//   1. Keep speed/omega/alpha conservative until in-place turns are repeatable.
//   2. Tune ENTRY_* for the start of the arc. Too small starts late and clips
//      the inside wall; too large starts early and drifts wide.
//   3. Tune EXIT_* so the robot finishes the arc aimed down the next cell and
//      reaches SENSING_POSITION_MM without a lateral step.
//   4. Tune OMEGA/ALPHA only after geometry is close. Higher omega shortens the
//      radius at a fixed speed; higher alpha makes the heading transition
//      sharper and can expose IMU/controller lag.
//   5. Tune left and right independently. Tire loading, motor asymmetry, and
//      ToF mounting make the two sides differ.
//   6. Raise SPEED last, and re-check the radius still fits a half cell.
//
// Not yet implemented: starting the turn from a front-ToF reading rather than
// from commanded distance — the turn begins where the caller arms it. The
// constants it would need are deliberately absent: a tuneable nothing reads
// is worse than no tuneable at all.

#endif // CONFIG_SMOOTH_TURN_H
