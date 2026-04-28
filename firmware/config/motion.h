/**
 * @file motion.h
 * @brief Speed limits, accelerations, and control behavior
 */
#ifndef CONFIG_MOTION_H
#define CONFIG_MOTION_H

// ===================== Control Loop ===================== //
#define LOOP_FREQUENCY_HZ 100.0f
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
#define ROBOT_YAW_TOLERANCE_DEG           0.5f  // Turn done when error < this
#define ROBOT_STOPPING_VELOCITY_MMPS      50.0f // Move done when speed < this
#define ROBOT_TURN_STABILITY_DEGPS        3.0f  // Turn stable when omega < this
#define ROBOT_FORWARD_SETTLE_TOLERANCE_MM 3.0f  // Move done when |measured - target| < this
#define ROBOT_FORWARD_SETTLE_TIMEOUT_MS   400   // Bound on settling phase after profile finishes

// ================= Control Output Limits =============== //
#define ROBOT_MAX_DUTY               1.0f  // Max PWM duty cycle
#define ROBOT_MAX_VOLTS_SLEW_PER_SEC 60.0f // Voltage rate limit (V/s)

// ================== Wall Interaction =================== //
#define WALL_CONTACT_THRESHOLD_MM 30.0f // Distance = touching wall
#define CENTERING_CORRECTION_GAIN 0.01f // Lateral wall centering gain

#endif // CONFIG_MOTION_H
