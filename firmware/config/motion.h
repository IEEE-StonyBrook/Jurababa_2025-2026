/**
 * @file motion.h
 * @brief Speed limits, accelerations, and control behavior
 */
#ifndef CONFIG_MOTION_H
#define CONFIG_MOTION_H

// ===================== Control Loop ===================== //
// Forward (encoder-paced) runs at 500 Hz: matches mazerunner-core / motorlab,
// gives ~26x bandwidth margin over closed-loop omega_n.
// Rotation (IMU-paced) runs at 100 Hz: BNO085 in RVC mode emits at exactly
// 100 Hz, so 500 Hz on rotation feedback would alias the D-term. Rotation PID
// gates on imu->has_new_yaw_sample() and uses a zero-order hold between packets.
#define LOOP_FREQUENCY_HZ 500.0f
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ)

// Rotation PID rate — locked to BNO085 RVC packet cadence.
#define ROTATION_LOOP_HZ    100.0f
#define ROTATION_INTERVAL_S (1.0f / ROTATION_LOOP_HZ)

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
#define ROBOT_MAX_SEARCH_SPEED_MMPS 300.0f // Search mode cruise speed
#define ROBOT_BACKUP_SPEED_MMPS     100.0f // Reverse speed

// ================ Forward Acceleration ================= //
#define ROBOT_BASE_ACCEL_MMPS2 1500.0f

// ================= Rotation Limits ===================== //
// Mirrors mazerunner-core Orion: OMEGA_SPIN_TURN = 360 deg/s, ALPHA_SPIN_TURN
// = 3600 deg/s^2. With alpha = 720 the trapezoid was triangular at 90 deg
// (peak omega clipped to ~254 deg/s) and an in-place 90 took ~0.7 s. At 3600
// the ramp-up reaches the 360 cap in 0.10 s and the full 90 completes in
// ~0.35 s — the UKMARSBOT-class hardware sustains this comfortably.
#define ROBOT_MAX_TURN_SPEED_DEGPS       360.0f  // Max angular velocity
#define ROBOT_BASE_ANGULAR_ACCEL_DEGPS2  3600.0f // Turn acceleration
#define ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS 250.0f  // Linear speed during smooth turns

// =============== Completion Tolerances ================= //
// Used by line_follower yaw-snap; main Robot now relies purely on profile.finished().
#define ROBOT_YAW_TOLERANCE_DEG 0.5f

// ================== Wall Interaction =================== //
#define CENTERING_CORRECTION_GAIN 0.01f // Lateral wall centering gain

#endif // CONFIG_MOTION_H
