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
// Filtered front ToF at the true cell centre (CENTER + wall ahead, S snapshot):
// 27-28 mm typical, rare 34 mm. Near the VL53L0X low limit but repeatable; the
// chip offset is uncalibrated, so this is a matched reading, not true mm.
constexpr float FRONT_REFERENCE_MM              = 28.0f;
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
