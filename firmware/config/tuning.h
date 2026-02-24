/**
 * @file tuning.h
 * @brief Motor feedforward and PID controller gains
 *
 * These values are empirically tuned. Use MotorLab to characterize
 * motors and determine optimal gains.
 */
#ifndef CONFIG_TUNING_H
#define CONFIG_TUNING_H

// ================= Feedforward Constants ================= //
// Duty = Kv * velocity + Ks + Ka * acceleration
// Determined from MotorLab open-loop trials

// Forward direction
#define FORWARD_KVL 0.000422394f // Left velocity gain (duty per mm/s)
#define FORWARD_KVR 0.000422394f // Right velocity gain
#define FORWARD_KSL 4.4512f      // Left static friction (duty)
#define FORWARD_KSR 4.4512f      // Right static friction

// Reverse direction (may differ due to motor asymmetry)
#define REVERSE_KVL 0.000422394f // Left velocity gain reverse
#define REVERSE_KVR 0.000422394f // Right velocity gain reverse
#define REVERSE_KSL 4.4512f      // Left static friction reverse
#define REVERSE_KSR 4.4512f      // Right static friction reverse

// Acceleration feedforward (tune after step response tests)
#define FORWARD_KAL 0.0f // Left acceleration gain (duty per mm/s²)
#define FORWARD_KAR 0.0f // Right acceleration gain
#define REVERSE_KAL 0.0f
#define REVERSE_KAR 0.0f

// ================= Position PD Controller ================= //
// Forward position control (mazerunner-core pattern)
#define FWD_KP 2.0f // Forward position proportional gain
#define FWD_KD 1.1f // Forward position derivative gain

// Rotation position control
#define ROT_KP 0.15f // Rotation proportional gain
#define ROT_KD 0.5f  // Rotation derivative gain

// ================= Wall Centering ================= //
#define CENTERING_CORRECTION_GAIN 0.01f // Lateral centering gain

// ================= Line Follower PD ================= //
#define LINE_KP                     0.3f   // Line position proportional gain
#define LINE_KD                     0.1f   // Line position derivative gain
#define LINE_FOLLOW_BASE_SPEED_MMPS 150.0f // Forward speed while following

#endif // CONFIG_TUNING_H
