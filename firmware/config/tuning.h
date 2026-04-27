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
#define MOTOR_KM 126.58f  // mm/s per volt  (from OL trial)
#define MOTOR_TM 0.08456f // seconds        (from STEP trial)

// ================ Feedforward (per motor) ============== //
// V = kV * speed + kS + kA * accel
#define FORWARD_KVL 1.0f / 127.90f // Left  V/(mm/s)     = 1/kM
#define FORWARD_KVR 1 / 125.10f    // Right V/(mm/s)
#define FORWARD_KSL 0.6323f        // Left  static friction (V)
#define FORWARD_KSR 0.5516f        // Right static friction (V)
#define FORWARD_KAL 0.0006611f     // Left  V/(mm/s^2)   = Tm/kM
#define FORWARD_KAR 0.0006757f     // Right V/(mm/s^2)

// ================ Forward PD Controller ================ //
// Design params: pick zeta & Td → auto-derive kP, kD
//   kP = Tm / (kM * Td^2)              (standard 2nd-order pole placement)
//   kD = (2 * zeta * Tm / Td - 1) / kM (positive when Td < 2*zeta*Tm)
#define FWD_ZETA 0.707f                                // Damping ratio
#define FWD_TD   (MOTOR_TM)                            // Natural period 1/omega_n
#define FWD_KP   (1.0f / (MOTOR_KM * MOTOR_TM))        // ≈ 1/(kM*Tm) ≈ 0.0935
#define FWD_KD   ((2.0f * FWD_ZETA - 1.0f) / MOTOR_KM) // ≈ 0.414/kM ≈ 0.00327

// ================ Rotation PD Controller =============== //
#define ROT_KP 0.01f // Turn proportional gain
#define ROT_KD 0.0f  // Turn derivative gain

// =================== Line Follower ===================== //
#define LINE_KP                     0.3f
#define LINE_KD                     0.1f
#define LINE_FOLLOW_BASE_SPEED_MMPS 150.0f

#endif // CONFIG_TUNING_H
