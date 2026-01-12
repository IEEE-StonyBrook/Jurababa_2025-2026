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
#define FORWARD_KVL  0.000410f  // Left velocity gain (duty per mm/s)
#define FORWARD_KVR  0.000382f  // Right velocity gain
#define FORWARD_KSL  0.505f     // Left static friction (duty)
#define FORWARD_KSR  0.52f      // Right static friction

// Reverse direction (may differ due to motor asymmetry)
#define REVERSE_KVL  0.000410f  // Left velocity gain reverse
#define REVERSE_KVR  0.000382f  // Right velocity gain reverse
#define REVERSE_KSL  0.51f      // Left static friction reverse
#define REVERSE_KSR  0.525f     // Right static friction reverse

// Acceleration feedforward (tune after step response tests)
#define FORWARD_KAL  0.0f       // Left acceleration gain (duty per mm/s²)
#define FORWARD_KAR  0.0f       // Right acceleration gain
#define REVERSE_KAL  0.0f
#define REVERSE_KAR  0.0f

// ================= Position PD Controller ================= //
// Forward position control (mazerunner-core pattern)
#define FWD_KP  0.05f   // Forward position proportional gain
#define FWD_KD  0.6f    // Forward position derivative gain

// Rotation position control
#define ROT_KP  0.15f   // Rotation proportional gain
#define ROT_KD  0.5f    // Rotation derivative gain

// ================= Wheel Velocity PID ================= //
// Legacy velocity controller (if used)
#define LEFT_WHEEL_KP   0.00015f
#define RIGHT_WHEEL_KP  0.00015f
#define LEFT_WHEEL_KI   0.0375f
#define RIGHT_WHEEL_KI  0.0375f
#define LEFT_WHEEL_KD   0.00015f
#define RIGHT_WHEEL_KD  0.00015f

// Yaw hold controller
#define YAW_KP  8.0f
#define YAW_KI  0.0f
#define YAW_KD  1.0f

// ================= Wall Centering ================= //
#define CENTERING_CORRECTION_GAIN  0.01f  // Lateral centering gain

#endif  // CONFIG_TUNING_H
