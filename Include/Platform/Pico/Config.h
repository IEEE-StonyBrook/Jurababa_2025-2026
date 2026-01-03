#ifndef CONFIG_H
#define CONFIG_H

// Control loop frequency and period.
#define LOOP_FREQUENCY_HZ 100.0f                     // Control updates per second.
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ) // Seconds per update.

// Encoder and wheel geometry.
#define WHEEL_DIAMETER_MM     39.8f     // Wheel diameter.
#define TICKS_PER_REVOLUTION  359.7222f // Encoder ticks per wheel turn.
#define MM_PER_TICK           ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION) // Distance per tick.
#define TO_CENTER_DISTANCE_MM ((167.5f - WHEEL_DIAMETER_MM) / 2.0f)

// Robot geometry.
#define WHEEL_BASE_MM 81.95f // Distance between left and right wheels.
#define DEG_PER_MM_DIFFERENCE                                                                      \
    (180.0f / (3.14159265f * WHEEL_BASE_MM)) // Deg per mm difference between wheels.

// Motor control and PWM.
#define PWM_WRAP        999u   // Max PWM counter (resolution).
#define MIN_DUTY_0_TO_1 0.225f // Minimum duty cycle to move motor.
#define MAX_VOLTAGE     6.0f   // Max safe motor voltage.

// Battery reference.
#define DEFAULT_BATTERY_VOLTAGE 8.35f // Nominal charged battery voltage.

// Feedforward constants.
#define FORWARD_KVL 0.000410f // Left duty gain forward (duty per mm/s)
#define FORWARD_KVR 0.000382f // Right duty gain forward (duty per mm/s)
#define FORWARD_KSL 0.505f    // Left static friction duty forward (duty)
#define FORWARD_KSR 0.52f     // Right static friction duty forward (duty)

#define REVERSE_KVL 0.000410f // TEMP: set after reverse sweep fit
#define REVERSE_KVR 0.000382f // TEMP: set after reverse sweep fit
#define REVERSE_KSL 0.51f     // TEMP: set after reverse sweep fit
#define REVERSE_KSR 0.525f    // TEMP: set after reverse sweep fit

// PID constants.
// #define LEFT_WHEEL_KP  0.0075f / 10.00f
// #define RIGHT_WHEEL_KP 0.0075f / 10.00f
#define LEFT_WHEEL_KP  0.00015f
#define RIGHT_WHEEL_KP 0.00015f
#define LEFT_WHEEL_KI  0.0375f
#define RIGHT_WHEEL_KI 0.0375f
#define LEFT_WHEEL_KD  0.00015f
#define RIGHT_WHEEL_KD 0.00015f
#define YAW_KP         8.0f
#define YAW_KI         0.0f
#define YAW_KD         1.0f

// ================== Motion Constants ================== //
// Distances.
#define CELL_DISTANCE_MM      180.0f                    // One cell length in mm.
#define HALF_CELL_DISTANCE_MM (CELL_DISTANCE_MM / 2.0f) // Half cell length in mm.

// Linear speed/acceleration.
#define FORWARD_TOP_SPEED   300.0f // Max forward speed (mm/s).
#define FORWARD_FINAL_SPEED 0.0f   // End speed for forward motions.
#define FORWARD_ACCEL       200.0f // Forward accel (mm/s^2).

// Rotational speed/acceleration.
#define TURN_TOP_SPEED   100.0f // Max angular speed (deg/s).
#define TURN_FINAL_SPEED 0.0f   // End angular speed.
#define TURN_ACCEL       50.0f  // Angular accel (deg/s^2).

// ================= ARC TURN CONFIGURATION ================= //
// Arc turning parameters for smooth turns (replaces in-place pivots)

// Linear velocity during arc turns (mm/s)
// Conservative starting value: 300 mm/s (0.3 m/s)
// Can increase after testing stability
#define ARC_TURN_VELOCITY_MMPS 300.0f

// Centripetal acceleration limit (mm/s²)
// Physics: a_c = v² / R, so R = v² / a_c
// With 300 mm/s and 1500 mm/s²: R = 60mm (tight but feasible)
#define ARC_CENTRIPETAL_ACCEL_MMPS2 1500.0f

// Turn radius calculation (derived from above)
// R = v² / a_c = (300²) / 1500 = 60mm
#define ARC_TURN_RADIUS_MM                                                                         \
    ((ARC_TURN_VELOCITY_MMPS * ARC_TURN_VELOCITY_MMPS) / ARC_CENTRIPETAL_ACCEL_MMPS2)

// Arc length for standard turns (calculated from radius)
// For 90°: L = R × π/2 = 60 × 1.5708 ≈ 94mm
// For 45°: L = R × π/4 = 60 × 0.7854 ≈ 47mm
#define ARC_90_DEGREE_LENGTH_MM (ARC_TURN_RADIUS_MM * 1.5708f) // π/2 ≈ 1.5708
#define ARC_45_DEGREE_LENGTH_MM (ARC_TURN_RADIUS_MM * 0.7854f) // π/4 ≈ 0.7854

// Completion tolerance for arc turns
#define ARC_TURN_DISTANCE_TOLERANCE_MM 3.0f // Arc length completion threshold
#define ARC_TURN_YAW_TOLERANCE_DEG     1.0f // Yaw accuracy (looser than in-place 0.5°)

// Velocity profiles (differentiated speeds for different maneuvers)
#define VELOCITY_PROFILE_STRAIGHT_MMPS 500.0f // Speed during straights (faster)
#define VELOCITY_PROFILE_TURN_MMPS     300.0f // Speed during turns (conservative)

// ================= IMU UART CONFIG ================= //
#define IMU_UART_ID   uart1            // UART interface used by IMU.
#define IMU_UART_IRQ  UART1_IRQ        // Interrupt request line for IMU UART.
#define IMU_BAUD_RATE 115200           // Baud rate for IMU communication.
#define IMU_DATA_BITS 8                // Number of data bits per frame.
#define IMU_STOP_BITS 1                // Stop bit length.
#define IMU_PARITY    UART_PARITY_NONE // No parity check.

// ================= IMU PACKET FORMAT (BNO085 RVC) =============== //
#define IMU_PACKET_LEN   19 // Total bytes in each IMU packet.
#define IMU_IDX_HDR0     0  // Start-of-packet header (0xAA).
#define IMU_IDX_HDR1     1  // Report ID (0x01 = rotation vector).
#define IMU_IDX_YAW_L    2  // Yaw low byte.
#define IMU_IDX_YAW_H    3  // Yaw high byte.
#define IMU_IDX_PITCH_L  4  // Pitch low byte.
#define IMU_IDX_PITCH_H  5  // Pitch high byte.
#define IMU_IDX_ROLL_L   6  // Roll low byte.
#define IMU_IDX_ROLL_H   7  // Roll high byte.
#define IMU_CHKSUM_FIRST 2  // First byte included in checksum.
#define IMU_CHKSUM_LAST  17 // Last byte included in checksum.
#define IMU_IDX_CHECKSUM 18 // Checksum byte (XOR of bytes 2 to 14).

// Expected header bytes for BNO085 RVC packets.
#define IMU_HDR0 170

// ================= Robot Control Constants ================= //
// High-level motion control tuning parameters

// Velocity limits and thresholds
#define ROBOT_MAX_WHEEL_SPEED_MMPS      800.0f // Maximum wheel speed (mm/s)
#define ROBOT_MAX_YAW_DIFF_MMPS         300.0f // Maximum yaw correction differential (mm/s)
#define ROBOT_MAX_ANGULAR_VEL_DEGPS     360.0f // Maximum turning speed (deg/s)
#define ROBOT_MIN_CRUISE_VELOCITY_MMPS  80.0f  // Minimum cruise speed before final deceleration
#define ROBOT_FINAL_APPROACH_SPEED_MMPS 30.0f  // Very low speed for final positioning
#define ROBOT_MIN_TURN_SPEED_MMPS       50.0f  // Minimum differential speed for turning

// Acceleration and deceleration
#define ROBOT_BASE_ACCEL_MMPS2 1500.0f // Linear acceleration rate (mm/s²)

// Completion and tolerance thresholds
#define ROBOT_YAW_TOLERANCE_DEG       0.5f  // Yaw error tolerance for motion completion (degrees)
#define ROBOT_YAW_ERROR_THRESHOLD_DEG 0.5f  // Minimum error to apply turn correction (degrees)
#define ROBOT_STOPPING_DISTANCE_MM    2.0f  // Distance threshold for motion completion (mm)
#define ROBOT_STOPPING_VELOCITY_MMPS  50.0f // Velocity threshold for motion completion (mm/s)
#define ROBOT_TURN_STABILITY_DEGPS    3.0f  // Angular velocity threshold for turn stability (deg/s)

// Control loop limits
#define ROBOT_MAX_DUTY              1.0f  // Maximum PWM duty cycle
#define ROBOT_MAX_DUTY_SLEW_PER_SEC 10.0f // Maximum duty cycle change per second

// ================= Drivetrain Constants ================= //
#define DRIVETRAIN_MIN_DT           0.001f // Minimum valid time step for velocity calculation (s)
#define DRIVETRAIN_FF_DEADZONE_MMPS 10.0f  // Feedforward deadzone to prevent motor buzzing (mm/s)
#define DRIVETRAIN_MAX_VELOCITY_MMPS                                                               \
    1500.0f // Sanity check threshold for velocity calculation (mm/s)

// ================= Sensor Constants ================= //
#define SENSORS_ANGULAR_VEL_FILTER_ALPHA                                                           \
    0.7f // Low-pass filter alpha for angular velocity smoothing

// ================= IMU Constants ================= //
#define IMU_RAW_TO_DEGREES_DIVISOR 100.0f // Conversion factor from raw IMU value to degrees

// ================= ToF Sensor Constants ================= //
#define TOF_TIMING_BUDGET_US        20000 // ToF measurement timing budget (microseconds)
#define TOF_MEASUREMENT_PERIOD_MS   30    // ToF inter-measurement period (milliseconds)
#define TOF_CELL_DEPTH_TO_CHECK_MM  40    // Depth to check for walls
#define TOF_LEFT_WALL_THRESHOLD_MM  100   // Left wall detection threshold
#define TOF_RIGHT_WALL_THRESHOLD_MM 100   // Right wall detection threshold
#define TOF_FRONT_WALL_THRESHOLD_MM 120   // Front wall detection threshold
#define TOF_MAX_RANGE_MM            500   // Maximum ToF sensor range (mm)

// ================= Mazerunner-Core Position Control ================= //
// Forward PD controller gains (position control, not velocity!)
#define FWD_KP  0.05f  // Proportional gain for forward position error
#define FWD_KD  0.6f   // Derivative gain for forward position error

// Rotation PD controller gains (angle control)
#define ROT_KP  0.15f  // Proportional gain for rotation error
#define ROT_KD  0.5f   // Derivative gain for rotation error

// Acceleration feedforward constants (Ka term) - Start at 0, tune empirically
// TODO: Tune these values after testing - typical range 0.0001 to 0.001
#define FORWARD_KAL  0.0f  // Left motor acceleration gain (duty per mm/s²)
#define FORWARD_KAR  0.0f  // Right motor acceleration gain
#define REVERSE_KAL  0.0f  // Left motor reverse acceleration gain
#define REVERSE_KAR  0.0f  // Right motor reverse acceleration gain

// Cell navigation constants
#define CELL_SIZE_MM  180.0f  // Standard micromouse cell size (mm)

// Motion speed presets (mazerunner-core style)
#define ROBOT_MAX_SEARCH_SPEED_MMPS       300.0f  // Search mode speed (mm/s)
#define ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS  250.0f  // Arc turn speed (mm/s)
#define ROBOT_MAX_TURN_SPEED_DEGPS        360.0f  // Turn speed (deg/s) - alias for clarity
#define ROBOT_BASE_ANGULAR_ACCEL_DEGPS2   720.0f  // Angular acceleration (deg/s²)
#define ROBOT_BACKUP_SPEED_MMPS           100.0f  // Backup/reverse speed (mm/s)

// Wall calibration constants
#define WALL_CONTACT_THRESHOLD_MM  30.0f  // Distance indicating wall contact (mm)
#define BACK_WALL_TO_CENTER_MM     90.0f  // Distance from back wall to cell center (mm)

// Wall following and centering
#define CENTERING_CORRECTION_GAIN  0.01f  // Lateral centering gain (dimensionless)

// ================= Multicore Constants ================= //
#define CORE_SLEEP_MS 250 // Sleep time for core loops (ms)

#endif