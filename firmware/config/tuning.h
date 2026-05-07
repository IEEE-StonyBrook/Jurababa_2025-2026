/**
 * @file tuning.h
 * @brief Calibration values from DriverLab trials
 *
 * Workflow:  OL → STEP → MOVE → TURN-OL → TURN-STEP
 *   1. OL        → kM, kS per motor (forward)
 *   2. STEP      → Tm (auto-calculates kV, kA)
 *   3. MOVE      → verify FWD_KP/FWD_KD formula gains
 *   4. TURN-OL   → ROT_KM (deg/s per volt of differential drive)
 *   5. TURN-STEP → ROT_TM (rotational time constant)
 *
 * All PD gains are formula-derived from the motor model (Km, Tm) and design
 * parameters (zeta, Td). Per mazerunner-core's modern control design — no
 * hand-tuning. If a gain feels wrong, the FIX is to re-measure Km/Tm in
 * DriverLab, not to nudge KP/KD by hand.
 */
#ifndef CONFIG_TUNING_H
#define CONFIG_TUNING_H

// ===================== Motor Model ===================== //
// Forward (linear) motor model — from OL + STEP trials.
#define MOTOR_KM 317.06f // mm/s per volt  (steady-state gain)
#define MOTOR_TM 0.080f  // seconds        (time constant)

// Rotational motor model — from TURN-OL + TURN-STEP trials.
#define ROT_KM 244.58f // deg/s per volt of differential drive
#define ROT_TM 0.128f  // seconds, rotational time constant

// ================ Feedforward (per motor) ============== //
// V = kV * speed + kS + kA * accel (per wheel; characterized independently).
// Per-wheel asymmetry preserved: the L/R difference (~10% in kS) is real.
#define FORWARD_KVL (1.0f / MOTOR_KM) // Left  V/(mm/s)     = 1/kM
#define FORWARD_KVR (1.0f / MOTOR_KM) // Right V/(mm/s)
#define FORWARD_KSL 0.6907f           // Left  static friction (V)
#define FORWARD_KSR 0.5701f           // Right static friction (V)
#define FORWARD_KAL 0.0002194f        // Left  V/(mm/s^2)   = Tm/kM
#define FORWARD_KAR 0.0002194f        // Right V/(mm/s^2)

// ================ Forward PD Controller ================ //
// Mazerunner-core / motorlab formulation:
//   FWD_KP = 16 * Tm / (Km * zeta^2 * Td^2)
//   FWD_KD = (8 * Tm - Td) / (Km * Td)
//
// PD controller is u = Kp * e + Kd * (e - e_old). Because this is a
// sampled system the error change is scaled by the sample time — rather
// than dividing in the loop we pre-multiply Kd by LOOP_FREQUENCY (see
// firmware/control/pid.cpp).
//
// Td = Tm here (NOT Tm/2 as in mazerunner-core / motorlab). Reason:
// Jurababa's gear ratio (29.86) gives Tm ≈ 0.05 s — about 4x snappier
// than UKMARSBOT's plant. With Td = Tm/2 the formula yields kP ≈ 7,
// which saturates on a single-tick of encoder noise (0.37 mm/tick at
// 500 Hz) and produces buzzing rather than the smooth motion that the
// same formula gives on UKMARSBOT. Setting Td = Tm matches the closed-
// loop bandwidth to the motor itself instead of trying to double it,
// landing kP near UKMARS-class values (~1.8 V per mm/s).
#define FWD_ZETA 0.707f
#define FWD_TD   (MOTOR_TM)
#define FWD_KP   (16.0f * MOTOR_TM / (MOTOR_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD))
#define FWD_KD   ((8.0f * MOTOR_TM - FWD_TD) / (MOTOR_KM * FWD_TD))

// ================ Rotation PD Controller =============== //
// Mazerunner-core / motorlab formulation applied to the measured rotational
// plant. DriverLab TURN uses the same 500 Hz PID convention as the forward
// loop: kD is multiplied by LOOP_FREQUENCY inside PID::update().
#define ROT_ZETA 0.707f
#define ROT_TD   0.400f
#define ROT_KP   (16.0f * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD))
#define ROT_KD   ((8.0f * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD))

// =================== Line Follower ===================== //
// Line position units are sensor slots: -3.5 at X8/left, +3.5 at X1/right.
// The controller negates line position so a line left of center commands
// positive omega (CCW/left), matching Jurababa's rotation convention.
//
// The line plant is geometric (sensor centroid → robot pose), NOT a 1st-
// order motor. Gains are tuned empirically, not via the Tm/Td/zeta formula
// used for FWD_*/ROT_*. The outer loop emits an omega offset consumed by
// Motion::set_line_steering_adjustment_degps and a forward target speed
// consumed by Motion::set_target_velocity.
//
// Defaults below are *reasonable working values* — target 400 mm/s with
// speed scheduling and gain scheduling enabled. Verify behavior at this
// point before walking the constants up to the competition profile shown
// in the comment block. Ramp procedure is documented in line_follower.cpp.
//
// ───────────────── Competition profile (override after verifying) ─────
//   LINE_TARGET_SPEED_MMPS         700.0f
//   LINE_MAX_SPEED_MMPS            900.0f
//   LINE_MIN_SPEED_MMPS            300.0f
//   LINE_GAIN_REF_SPEED_MMPS       500.0f
//   LINE_GAIN_SCHED_FLOOR_MMPS     250.0f
//   LINE_KP_BASE_DEGPS_PER_SLOT    90.0f
//   LINE_KD_BASE_DEG_PER_SLOT      6.5f
//   LINE_OMEGA_LIMIT_DEGPS         800.0f
//   LINE_BRAKE_GAIN_MMPS_PER_SLOT  160.0f
//   LINE_LOST_HOLD_MS              60
//   LINE_LOST_STOP_MS              200
//   LINE_RECOVERY_AUTHORITY        0.7f
//   LINE_BRANCH_STEER_BIAS_DEGPS   280.0f
//   LINE_BRANCH_CAPTURE_MS         150
//   LINE_INTERSECTION_LOCKOUT_MS   150
// ──────────────────────────────────────────────────────────────────────

// Speed envelope (forward target velocity, mm/s)
#define LINE_TARGET_SPEED_MMPS      400.0f
#define LINE_MAX_SPEED_MMPS         600.0f
#define LINE_MIN_SPEED_MMPS         200.0f
#define LINE_FOLLOW_RUN_DISTANCE_MM 100000.0f

// PD steering — gain-scheduled vs current forward velocity so the
// per-millimeter response stays consistent across the speed envelope.
#define LINE_GAIN_REF_SPEED_MMPS    350.0f
#define LINE_GAIN_SCHED_FLOOR_MMPS  200.0f
#define LINE_KP_BASE_DEGPS_PER_SLOT 70.0f
#define LINE_KD_BASE_DEG_PER_SLOT   4.5f
#define LINE_OMEGA_LIMIT_DEGPS      500.0f

// Error filter + predictive lookahead (substitutes for physical sensor
// mounting offset ahead of the wheel axle).
#define LINE_ERROR_FILTER_ALPHA 0.45f
#define LINE_LOOKAHEAD_TIME_S   0.035f

// Speed scheduling on |error|: target_v = TARGET - BRAKE * |e_filt|,
// clamped to [MIN, MAX]. With BRAKE=130 and TARGET=400, |e|=1 drops
// target to 270 mm/s, |e|=2 drops to MIN.
#define LINE_BRAKE_GAIN_MMPS_PER_SLOT 130.0f

// Line loss / recovery — slightly forgiving timings while you're
// confirming the new control law tracks cleanly.
#define LINE_LOST_HOLD_MS       100
#define LINE_LOST_STOP_MS       300
#define LINE_RECOVERY_AUTHORITY 0.6f

// Intersection / route (kept from existing design).
#define LINE_BRANCH_STEER_BIAS_DEGPS 200.0f
#define LINE_BRANCH_CAPTURE_MS       200
#define LINE_INTERSECTION_LOCKOUT_MS 200

#endif // CONFIG_TUNING_H
