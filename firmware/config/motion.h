/**
 * @file motion.h
 * @brief Speed limits, accelerations, and control behavior
 */
#ifndef CONFIG_MOTION_H
#define CONFIG_MOTION_H

// ===================== Control Loop ===================== //
// Forward and rotation controllers run at 500 Hz, matching mazerunner-core /
// motorlab. Jurababa's rotation feedback still comes from the 100 Hz BNO085
// RVC stream; IMU::robot_rot_change() distributes the latest packet-rate yaw
// delta into an equivalent per-500 Hz tick change so the controller keeps the
// UKMARS single-rate shape without switching away from the IMU.
#define LOOP_FREQUENCY_HZ 500.0f
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ)

// 8-tap moving average over per-tick encoder deltas, applied in
// Drivetrain::update() and DriverLab::sampleEncoders() to mirror
// ukmars/motorlab/src/encoders.h:108. Adds ~16 ms phase lag at 500 Hz
// in exchange for quantization rejection on the D-term and a clean plot
// trace. Forward PD gains are tuned against this smoothed input.
//
// CAUTION (per Peter Harrison's encoders.h header): changing this length
// adds delay to the feedback loop and forces re-tuning of the forward
// PD controller. Rotation PD is unaffected (uses IMU, not encoders).
#define ENCODER_AVERAGER_LENGTH 8

// ================ Forward Speed Limits ================= //
#define ROBOT_MAX_SEARCH_SPEED_MMPS  300.0f // Search mode cruise speed
#define ROBOT_SEARCH_TURN_SPEED_MMPS 250.0f // Constant speed through search smooth turns

// ================ Forward Acceleration ================= //
#define ROBOT_BASE_ACCEL_MMPS2 1500.0f

// ================= Rotation Limits ===================== //
// Jurababa-validated IMU spin-turn defaults. These are deliberately gentler
// than Orion-class UKMARS values because our rotation feedback comes from the
// BNO085 RVC stream rather than 500 Hz encoder yaw.
#define ROBOT_MAX_TURN_SPEED_DEGPS         240.0f  // Max angular velocity
#define ROBOT_BASE_ANGULAR_ACCEL_DEGPS2    800.0f  // Turn acceleration
#define ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS   250.0f  // Linear speed during smooth turns
#define ROBOT_SMOOTH_TURN_OMEGA_DEGPS      287.0f  // UKMARS SS90E starting point
#define ROBOT_SMOOTH_TURN_ALPHA_DEGPS2     2866.0f // UKMARS SS90E starting point
#define ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM 95.0f   // Starting ToF trigger calibration

// ================ Maze Heading Hold =================== //
// Competition maze-running can use the logical maze heading as a gentle
// steering correction through the existing rotation PID. This is separate from
// ToF wall steering: ToF may classify walls while heading hold uses IMU yaw to
// keep the physical mouse aligned to N/NE/E/... headings.
#define MAZE_HEADING_HOLD_ENABLE           1
#define MAZE_HEADING_HOLD_KP_DEGPS_PER_DEG 3.0f
#define MAZE_HEADING_HOLD_MAX_DEGPS        45.0f

// ================ Blind Path Test Defaults ============= //
// PATH defaults stay gentle so manually-entered race paths and RAW drivetrain
// benchmarks start safely.
#define CLI_PATH_SPEED_MMPS   180.0f
#define CLI_PATH_ACCEL_MMPS2  700.0f
#define CLI_PATH_OMEGA_DEGPS  240.0f
#define CLI_PATH_ALPHA_DEGPS2 800.0f

// =============== Completion Tolerances ================= //
// Used by line_follower yaw-snap; main Motion now relies purely on profile.finished().
#define ROBOT_YAW_TOLERANCE_DEG 0.5f

#endif // CONFIG_MOTION_H
