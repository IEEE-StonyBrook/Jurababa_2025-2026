/**
 * @file sensors.h
 * @brief Sensor hardware configuration and thresholds
 */
#ifndef CONFIG_SENSORS_H
#define CONFIG_SENSORS_H

// ================= ToF Sensor Configuration ================= //
// VL53L0X timing and measurement settings
#define TOF_TIMING_BUDGET_US      20000   // Measurement time budget (µs)
#define TOF_MEASUREMENT_PERIOD_MS 20      // Match timing budget for 50Hz updates
#define TOF_MAX_RANGE_MM          500     // Maximum reliable range (mm)
#define TOF_OUT_OF_RANGE_MM       8191.0f // VL53L0X invalid/open-space sentinel

// UKMARS mazerunner-core SIDE_NOMINAL pattern. Each side's raw mm reading
// at the centered-between-walls pose is captured here as a constant. The
// per-side scale factors normalize both sides to TOF_SIDE_NOMINAL so that
// downstream CTE math operates on a single symmetric unit, regardless of
// per-unit chip bias, mounting angle, or cover thickness.
//
// To recalibrate (after sensor swap or remount):
//   1. Boot DriverLab (press 'M' within 3 s of boot).
//   2. Center robot between two walls (1 mm symmetry is the goal; use
//      spacers if needed).
//   3. Run LTOF and RTOF — note the printed mm values.
//   4. Update TOF_LEFT_CALIBRATION_MM and TOF_RIGHT_CALIBRATION_MM here.
//   5. Rebuild, flash. Same centered pose should now produce ~0
//      side_error_norm (verify via S snapshot or steering CSV).
//
// The chip-level offset bias (±10–30 mm typical per ST UM2039) is fully
// absorbed by the scale: any value the chip outputs at the centered pose
// becomes "100" after multiplication. We don't need accurate mm; we need
// symmetric normalized units, which compile-time scaling delivers for free.
#define TOF_SIDE_NOMINAL         100.0f
#define TOF_LEFT_CALIBRATION_MM  120.0f // raw left ToF reading at center (re-measure!)
#define TOF_RIGHT_CALIBRATION_MM 100.0f // raw right ToF reading at center (re-measure!)
#define TOF_LEFT_SCALE           (TOF_SIDE_NOMINAL / TOF_LEFT_CALIBRATION_MM)
#define TOF_RIGHT_SCALE          (TOF_SIDE_NOMINAL / TOF_RIGHT_CALIBRATION_MM)

// Wall-detection thresholds stay in raw chip-mm. They're binary classifiers
// (wall present / absent) so per-unit bias doesn't change the decision; all
// it does is shift the mm threshold by 10–30 mm, which is well inside the
// margin between "centered between walls" (~100 mm) and "no wall there"
// (>>200 mm). Re-measure if the per-unit bias is unusually large.
#define TOF_SIDE_WALL_MARGIN_MM     40.0f
#define TOF_LEFT_WALL_THRESHOLD_MM  (TOF_LEFT_CALIBRATION_MM + TOF_SIDE_WALL_MARGIN_MM)
#define TOF_RIGHT_WALL_THRESHOLD_MM (TOF_RIGHT_CALIBRATION_MM + TOF_SIDE_WALL_MARGIN_MM)
#define TOF_FRONT_WALL_THRESHOLD_MM 110.0f

// UKMARS-style wall steering. Wall detection remains enabled regardless of
// this switch; TOF_STEERING_ENABLE only controls whether side-wall error is
// injected into the rotation PID. Keep disabled while search relies on pure
// forward PID + IMU yaw hold.
//
// Jurababa yaw/omega convention: positive yaw = CCW (left), so a positive
// side_error_norm (drifted right of center) commands a positive omega
// (steer left). Sign matches the IMU's IMU_YAW_SIGN inversion done once
// in the driver layer.
#define TOF_STEERING_ENABLE                 0
#define TOF_STEERING_KP_DEGPS_PER_NOMINAL   0.33f
#define TOF_STEERING_KD_DEG_PER_NOMINAL     0.0f
#define TOF_STEERING_ADJUST_LIMIT_DEGPS     10.0f
#define TOF_FRONT_WALL_RELIABILITY_LIMIT_MM 160.0f

// ================= IMU Configuration ================= //
// BNO085 UART settings
#define IMU_UART_ID   uart1
#define IMU_UART_IRQ  UART1_IRQ
#define IMU_BAUD_RATE 115200
#define IMU_DATA_BITS 8
#define IMU_STOP_BITS 1
#define IMU_PARITY    UART_PARITY_NONE

// BNO085 RVC packet format (19 bytes total)
// Byte layout: [0xAA][0xAA][Index][YawL][YawH][PitchL][PitchH][RollL][RollH]...
#define IMU_PACKET_LEN   19 // Total packet bytes
#define IMU_IDX_HDR0     0  // Header byte 0 (0xAA)
#define IMU_IDX_HDR1     1  // Header byte 1 (0xAA in RVC mode)
#define IMU_IDX_INDEX    2  // Packet index/sequence byte (skip this for data)
#define IMU_IDX_YAW_L    3  // Yaw low byte (was 2 - WRONG!)
#define IMU_IDX_YAW_H    4  // Yaw high byte (was 3 - WRONG!)
#define IMU_IDX_PITCH_L  5
#define IMU_IDX_PITCH_H  6
#define IMU_IDX_ROLL_L   7
#define IMU_IDX_ROLL_H   8
#define IMU_CHKSUM_FIRST 2   // First checksum byte index (includes index byte)
#define IMU_CHKSUM_LAST  17  // Last checksum byte index
#define IMU_IDX_CHECKSUM 18  // Checksum byte index
#define IMU_HDR0         170 // Expected header byte 0 (0xAA)
#define IMU_HDR1         170 // Expected header byte 1 (0xAA) - RVC uses 0xAA 0xAA

// IMU processing
#define IMU_RAW_TO_DEGREES_DIVISOR 100.0f

// BNO085 RVC packet rate. The chip emits packets at exactly 100 Hz; this is
// the authoritative cadence for omega estimation (delta_yaw * IMU_PACKET_HZ),
// NOT LOOP_FREQUENCY_HZ. Reading omega at the loop rate aliases the signal —
// 4 out of 5 ticks see "no change since last packet" and report 0.
#define IMU_PACKET_HZ 100.0f

// IMU yaw sign convention.
// Positive yaw must mean CCW (left turn) to match the rotation controller:
// left = forward - rotation, right = forward + rotation, where positive
// rotation_output spins the robot CCW. On this board the BNO085 is mounted
// such that CCW reads as negative on the chip, so we invert here once at the
// driver layer. Set to +1.0f if a future board mounts the IMU upright.
#define IMU_YAW_SIGN (-1.0f)

// ================= IMU Filtering ================= //
// Outlier guard: drop packets whose raw yaw step exceeds this. 20° per 10 ms =
// 2000°/s, which is past any physical turn rate Jurababa can produce — so any
// jump larger than this is a UART/I2C glitch, not motion.
#define IMU_MAX_YAW_DELTA_PER_SAMPLE 20.0f

// Moving-average window over per-packet yaw deltas. Mirrors the encoder path's
// IMU_DELTA_AVG_LENGTH = 1: passthrough, no averaging.
//
// Rationale: the encoder MA exists for *quantization* (per-tick encoder count
// is coarse). The IMU's per-packet yaw delta is 0.01° resolution and arrives
// pre-fused from the BNO085's internal Kalman filter — there's little raw
// noise left for an MA to reject, and a 4-tap MA at 100 Hz cost us 15 ms of
// group delay (vs UKMARS' 7 ms encoder MA at 500 Hz). With the rotation PD
// now single-rate at 500 Hz consuming `omega * dt`, the held-flat ZOH on
// rate already smooths the differential mixer; explicit MA on top of that
// is double-filtering. Re-raise to 2 if turn traces show packet-boundary
// jitter that costs more KP than the lag does. The ring buffer code below
// handles any N ≥ 1 correctly — no code change needed beyond this constant.
#define IMU_DELTA_AVG_LENGTH 1

// ================= Line Sensor Configuration ================= //
#define LINE_SENSOR_COUNT             8
#define LINE_SENSOR_I2C_ADDR          0x12 // YahBoom 8-channel sensor
#define LINE_SENSOR_DATA_REG          0x30 // Sensor data register
#define LINE_SENSOR_I2C_BAUD          100000
#define LINE_INTERSECTION_DEBOUNCE_MS 50

#endif // CONFIG_SENSORS_H
