/**
 * @file tuning.h
 * @brief Calibration values from DriverLab trials
 *
 * Workflow:  OL → STEP → MOVE → TURN
 *   1. OL    → kM, kS per motor
 *   2. STEP  → Tm (auto-calculates kV, kA)
 *   3. MOVE  → tune zeta/Td → auto kP, kD
 *   4. TURN  → tune ROT_KP, ROT_KD
 */
#ifndef CONFIG_TUNING_H
#define CONFIG_TUNING_H

// ===================== Motor Model ===================== //
#define MOTOR_KM 130.0f // mm/s per volt  (from OL trial)
#define MOTOR_TM 0.05f  // seconds        (from STEP trial)

// ================ Feedforward (per motor) ============== //
// V = kV * speed + kS + kA * accel
#define FORWARD_KVL 0.0029239766f // Left  V/(mm/s)     = 1/kM
#define FORWARD_KVR 0.0029239766f // Right V/(mm/s)
#define FORWARD_KSL 0.40f         // Left  static friction (V)
#define FORWARD_KSR 0.40f         // Right static friction (V)
#define FORWARD_KAL 0.0f          // Left  V/(mm/s^2)   = Tm/kM
#define FORWARD_KAR 0.0f          // Right V/(mm/s^2)

// ================ Forward PD Controller ================ //
// Design params: pick zeta & Td → auto-derive kP, kD
//   kP = 1 / (kM * Td)
//   kD = (2 * zeta * Td - Tm) / kM
#define FWD_ZETA 0.707f // Damping (0.707 = critical)
#define FWD_TD   0.025f // Derivative time (start at Tm/2)
#define FWD_KP   0.5f   // Proportional gain
#define FWD_KD   0.0f   // Derivative gain

// ================ Rotation PD Controller =============== //
#define ROT_KP 0.01f // Turn proportional gain
#define ROT_KD 0.0f  // Turn derivative gain

// =================== Line Follower ===================== //
#define LINE_KP                     0.3f
#define LINE_KD                     0.1f
#define LINE_FOLLOW_BASE_SPEED_MMPS 150.0f

#endif // CONFIG_TUNING_H
