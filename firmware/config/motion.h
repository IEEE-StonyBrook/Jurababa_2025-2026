/**
 * @file motion.h
 * @brief Speed limits, accelerations, and control behavior
 */
#ifndef CONFIG_MOTION_H
#define CONFIG_MOTION_H

// ===================== Control Loop ===================== //
// 500 Hz to match mazerunner-core: per-loop `KD * (e - e_prev)` formulation
// needs LOOP_FREQUENCY phase-lead, and ACC_FF scales with LOOP_FREQUENCY.
// ToF reads moved to Core 0 to keep the 2 ms tick budget; BNO085 stays at
// 100 Hz (input ceiling).
#define LOOP_FREQUENCY_HZ 500.0f
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ)

// ================ Forward Speed Limits ================= //
#define ROBOT_MAX_SEARCH_SPEED_MMPS 300.0f // Search mode cruise speed
#define ROBOT_BACKUP_SPEED_MMPS     100.0f // Reverse speed

// ================ Forward Acceleration ================= //
#define ROBOT_BASE_ACCEL_MMPS2 1500.0f

// ================= Rotation Limits ===================== //
#define ROBOT_MAX_TURN_SPEED_DEGPS       360.0f // Max angular velocity
#define ROBOT_BASE_ANGULAR_ACCEL_DEGPS2  720.0f // Turn acceleration
#define ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS 250.0f // Linear speed during smooth turns

// =============== Completion Tolerances ================= //
// Used by line_follower yaw-snap; main Robot now relies purely on profile.finished().
#define ROBOT_YAW_TOLERANCE_DEG 0.5f

// ================== Wall Interaction =================== //
#define CENTERING_CORRECTION_GAIN 0.01f // Lateral wall centering gain

#endif // CONFIG_MOTION_H
