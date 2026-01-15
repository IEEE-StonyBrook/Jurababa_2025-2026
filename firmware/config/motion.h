/**
 * @file motion.h
 * @brief Motion control speeds, accelerations, and tolerances
 *
 * Adjust these values to change robot behavior without touching control code.
 */
#ifndef CONFIG_MOTION_H
#define CONFIG_MOTION_H

// ================= Control Loop ================= //
#define LOOP_FREQUENCY_HZ 100.0f
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ)

// ================= Linear Motion ================= //
// Speed limits (mm/s)
#define FORWARD_TOP_SPEED               300.0f // Max forward speed
#define FORWARD_FINAL_SPEED             0.0f   // End speed for moves
#define ROBOT_MAX_SEARCH_SPEED_MMPS     300.0f // Search mode speed
#define ROBOT_BACKUP_SPEED_MMPS         100.0f // Reverse speed
#define ROBOT_MIN_CRUISE_VELOCITY_MMPS  80.0f  // Minimum cruise before decel
#define ROBOT_FINAL_APPROACH_SPEED_MMPS 30.0f  // Final positioning speed

// Acceleration (mm/s²)
#define FORWARD_ACCEL          200.0f
#define ROBOT_BASE_ACCEL_MMPS2 1500.0f

// ================= Rotational Motion ================= //
// Speed limits (deg/s)
#define TURN_TOP_SPEED              100.0f
#define TURN_FINAL_SPEED            0.0f
#define ROBOT_MAX_ANGULAR_VEL_DEGPS 360.0f
#define ROBOT_MAX_TURN_SPEED_DEGPS  360.0f // Alias for clarity

// Acceleration (deg/s²)
#define TURN_ACCEL                      50.0f
#define ROBOT_BASE_ANGULAR_ACCEL_DEGPS2 720.0f

// ================= Arc Turns ================= //
#define ARC_TURN_VELOCITY_MMPS      300.0f  // Linear speed during arc
#define ARC_CENTRIPETAL_ACCEL_MMPS2 1500.0f // Centripetal limit
#define ARC_TURN_RADIUS_MM                                                                         \
    ((ARC_TURN_VELOCITY_MMPS * ARC_TURN_VELOCITY_MMPS) / ARC_CENTRIPETAL_ACCEL_MMPS2)
#define ARC_90_DEGREE_LENGTH_MM          (ARC_TURN_RADIUS_MM * 1.5708f)
#define ARC_45_DEGREE_LENGTH_MM          (ARC_TURN_RADIUS_MM * 0.7854f)
#define ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS 250.0f

// Velocity profiles for path execution
#define VELOCITY_PROFILE_STRAIGHT_MMPS 500.0f
#define VELOCITY_PROFILE_TURN_MMPS     300.0f

// ================= Completion Tolerances ================= //
#define ROBOT_YAW_TOLERANCE_DEG        0.5f  // Turn completion (degrees)
#define ROBOT_YAW_ERROR_THRESHOLD_DEG  0.5f  // Min error for correction
#define ROBOT_STOPPING_DISTANCE_MM     2.0f  // Move completion (mm)
#define ROBOT_STOPPING_VELOCITY_MMPS   50.0f // Stop velocity threshold
#define ROBOT_TURN_STABILITY_DEGPS     3.0f  // Angular velocity for stable
#define ARC_TURN_DISTANCE_TOLERANCE_MM 3.0f
#define ARC_TURN_YAW_TOLERANCE_DEG     1.0f

// ================= Control Limits ================= //
#define ROBOT_MAX_WHEEL_SPEED_MMPS   800.0f // Absolute wheel speed limit
#define ROBOT_MAX_YAW_DIFF_MMPS      300.0f // Max differential for yaw
#define ROBOT_MIN_TURN_SPEED_MMPS    50.0f  // Min diff for turning
#define ROBOT_MAX_DUTY               1.0f
#define ROBOT_MAX_DUTY_SLEW_PER_SEC  10.0f
#define ROBOT_MAX_VOLTS_SLEW_PER_SEC 60.0f

// ================= Wall Calibration ================= //
#define WALL_CONTACT_THRESHOLD_MM 30.0f // Distance indicating contact
#define BACK_WALL_TO_CENTER_MM    90.0f // Back wall to cell center

#endif // CONFIG_MOTION_H
