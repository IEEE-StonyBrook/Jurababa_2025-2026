/**
 * @file sensors.h
 * @brief Sensor hardware configuration and thresholds
 */
#ifndef CONFIG_SENSORS_H
#define CONFIG_SENSORS_H

// ================= ToF Sensor Configuration ================= //
// VL53L0X timing and measurement settings
#define TOF_TIMING_BUDGET_US      20000 // Measurement time budget (µs)
#define TOF_MEASUREMENT_PERIOD_MS 20    // Match timing budget for 50Hz updates
#define TOF_MAX_RANGE_MM          500   // Maximum reliable range (mm)

// Wall detection thresholds (mm)
#define TOF_LEFT_WALL_THRESHOLD_MM  100
#define TOF_RIGHT_WALL_THRESHOLD_MM 100
#define TOF_FRONT_WALL_THRESHOLD_MM 120
#define TOF_CELL_DEPTH_TO_CHECK_MM  40

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

// IMU yaw sign convention.
// Positive yaw must mean CCW (left turn) to match the rotation controller:
// left = forward - rotation, right = forward + rotation, where positive
// rotation_output spins the robot CCW. On this board the BNO085 is mounted
// such that CCW reads as negative on the chip, so we invert here once at the
// driver layer. Set to +1.0f if a future board mounts the IMU upright.
#define IMU_YAW_SIGN (-1.0f)

// ================= IMU Filtering ================= //
#define IMU_YAW_FILTER_ALPHA         0.3f  // EMA filter for yaw (0.3 = moderate smoothing)
#define IMU_MAX_YAW_DELTA_PER_SAMPLE 20.0f // Max degrees change per 10ms (2000°/s physical limit)

// ================= Angular Velocity Filtering ================= //
#define SENSORS_ANGULAR_VEL_FILTER_ALPHA 0.15f // Was 0.7f - lower = more smoothing for omega

// ================= Line Sensor Configuration ================= //
#define LINE_SENSOR_COUNT             8
#define LINE_SENSOR_I2C_ADDR          0x12 // YahBoom 8-channel sensor
#define LINE_SENSOR_DATA_REG          0x30 // Sensor data register
#define LINE_SENSOR_I2C_BAUD          100000
#define LINE_INTERSECTION_DEBOUNCE_MS 50

#endif // CONFIG_SENSORS_H
