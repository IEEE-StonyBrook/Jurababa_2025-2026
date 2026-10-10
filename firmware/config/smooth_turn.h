/**
 * @file smooth_turn.h
 * @brief UKMARS-style smooth turn configuration.
 */
#ifndef CONFIG_SMOOTH_TURN_H
#define CONFIG_SMOOTH_TURN_H

#include <cstdint>

#include "config/geometry.h"
#include "config/motion.h"

// Smooth turn speed/omega/alpha and the entry/exit offsets are runtime
// tuneable over the CLI, so they live in Motion::SmoothTurnTuning (seeded from
// config/motion.h) rather than a constexpr table here.

constexpr float SENSING_POSITION_MM = CELL_SIZE_MM - 10.0f; // Osmium SENSING_POSITION = 170
// Largest heading error an arc corrects (Mouse::smooth_turn); search saw 15.
constexpr float SMOOTH_TURN_MAX_AIM_DEG = 20.0f;
// Dead-end 180s spin with the robot body, not the axle, centred in the cell:
// the axle sits 74 mm behind the nose, so an axle-centred spin swept the
// front corners into the walls (trace 2026-10-10). Body centred = nose ~37 mm
// off the wall, which the front ToF reads as ~49 (ruler: 30 -> 42, 40 -> 52).
constexpr float DEAD_END_SPIN_FRONT_MM = 49.0f;
// Body centre ahead of the axle: half the length less the axle-to-back offset.
constexpr float BODY_CENTRE_AHEAD_OF_AXLE_MM = ROBOT_LENGTH_MM / 2.0f - WHEEL_DIAMETER_MM / 2.0f;
// Front ToF at the true cell centre (nose ~10 mm off the wall). Ruler vs S,
// 2026-10-10: nose 0 mm reads 31, 10 -> 33, 20 -> 43, 30 -> 42, 40 -> 52. The
// old 28 was below the touching reading, so the stop crept into the wall.
constexpr float FRONT_REFERENCE_MM              = 33.0f;
constexpr float FRONT_CORRECTION_TOLERANCE_MM   = 15.0f; // UKMARS adjustPosition tolerance = 50 raw
constexpr float FRONT_CORRECTION_STEP_MM        = 20.0f; // UKMARS correction step = 10 mm
constexpr float FRONT_CORRECTION_SPEED_MMPS     = 100.0f;  // UKMARS adj_speed = 100 mm/s
constexpr float FRONT_CORRECTION_ACCEL_MMPS2    = 1000.0f; // UKMARS adj_accel = 1000 mm/s^2
constexpr float STOP_AT_CENTRE_FRONT_SPEED_MMPS = 30.0f; // UKMARS front-wall final_speed = 30 mm/s
constexpr uint32_t STOP_AT_CENTRE_FRONT_TIMEOUT_MS   = 500;  // UKMARS front-wall timeout = 500 ms
constexpr uint32_t STOP_AT_CENTRE_PROFILE_TIMEOUT_MS = 1000; // UKMARS no-wall timeout = 1000 ms

// Smooth-turn tuning notes
// ------------------------
// Same shape as ukmars/mazerunner-core Mouse::turn_smooth() (SS90E):
//   - roll at SPEED until the robot is ENTRY mm before the turn cell centre,
//   - arc 90 deg at OMEGA/ALPHA while still rolling,
//   - drive (end point - EXIT) mm, where the arc ends EXIT mm past the centre.
// ENTRY and EXIT are tuned reaches, not derived: the ALPHA ramps make the real
// reach longer than SPEED/OMEGA. Defaults are mazerunner-core's Orion values
// (300 mm/s, 280 deg/s, 4000 deg/s^2, entry 70 L / 55 R, exit 80).
//
// Live knobs: `SMOOTH`, `SMOOTH SET <name> <value>`, `SMOOTH DEFAULTS`,
// `SMOOTH RUN L|R`. Names: SPEED OMEGA ALPHA ENTRY_L EXIT_L ENTRY_R EXIT_R.
//
// Tune in this order, using `SMOOTH RUN L|R` over tape for each change:
//   1. Keep SPEED/OMEGA/ALPHA conservative until in-place turns are repeatable.
//      A search turn must end before SENSING_POSITION_MM: if `SMOOTH` reports
//      EXIT LEG CLAMPED, raise ALPHA (or OMEGA) before touching offsets.
//   2. ENTRY_*: too small starts late and swings wide; too large starts early
//      and clips the inside post.
//   3. EXIT_*: set so the robot finishes the arc centred in the next cell and
//      reaches SENSING_POSITION_MM without a lateral step.
//   4. Tune left and right independently.
//
// mazerunner's front-sensor trigger: with a wall ahead the arc starts once the
// front ToF reads FRONT_TRIG_L/R mm, if that comes before the odometry turn
// point (see ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM). ANGLE_L/R sets the arc size.

#endif // CONFIG_SMOOTH_TURN_H
