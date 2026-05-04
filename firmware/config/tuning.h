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
#define MOTOR_KM 360.46f  // mm/s per volt  (steady-state gain)
#define MOTOR_TM 0.09617f // seconds        (time constant)

// Rotational motor model — placeholders until TURN-OL + TURN-STEP run.
// Provisional ratios derived from mazerunner-core Orion config (rotational
// gain ≈ 1.6× forward, rotational time constant ≈ 1.1× forward). Re-calibrate
// before relying on closed-loop turning.
#define ROT_KM 580.0f // deg/s per volt of differential drive (TODO: calibrate)
#define ROT_TM 0.106f // seconds                              (TODO: calibrate)

// ================ Feedforward (per motor) ============== //
// V = kV * speed + kS + kA * accel (per wheel; characterized independently).
// Per-wheel asymmetry preserved: the L/R difference (~10% in kS) is real.
#define FORWARD_KVL (1.0f / 361.10f) // Left  V/(mm/s)     = 1/kM
#define FORWARD_KVR (1.0f / 359.82f) // Right V/(mm/s)
#define FORWARD_KSL 0.5630f          // Left  static friction (V)
#define FORWARD_KSR 0.5077f          // Right static friction (V)
#define FORWARD_KAL 0.0006680f       // Left  V/(mm/s^2)   = Tm/kM
#define FORWARD_KAR 0.0006680f       // Right V/(mm/s^2)

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
// Same formula structure as forward, applied to the rotational plant.
// Td = Tm — mirrors mazerunner-core Orion (`const float ROT_TD = ROT_TM`).
// motorlab uses Tm/2, but on Jurababa that quadruples ROT_KP and the IMU's
// 100 Hz / 4-tap-MA feedback can't damp the resulting ringing. Aligning Td
// with Tm (same choice we already made for forward) puts closed-loop
// bandwidth at the rotational plant's natural break frequency. Re-run
// TURN-STEP after any change here.
#define ROT_ZETA 0.707f
#define ROT_TD   (ROT_TM)
#define ROT_KP   (16.0f * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD))
#define ROT_KD   ((8.0f * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD))

// =================== Line Follower ===================== //
#define LINE_KP                     0.3f
#define LINE_KD                     0.1f
#define LINE_FOLLOW_BASE_SPEED_MMPS 150.0f

#endif // CONFIG_TUNING_H
