/**
 * @file sensors.h
 * @brief Sensor hardware configuration and thresholds
 */
#ifndef CONFIG_SENSORS_H
#define CONFIG_SENSORS_H

// ================= ToF Sensor Configuration ================= //
// VL53L0X timing and measurement settings
#define TOF_TIMING_BUDGET_US      20000 // Measurement time budget (µs)
#define TOF_MEASUREMENT_PERIOD_MS 30    // Inter-measurement period (ms)
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

// BNO085 RVC packet format
#define IMU_PACKET_LEN   19 // Total packet bytes
#define IMU_IDX_HDR0     0  // Header byte 0 (0xAA)
#define IMU_IDX_HDR1     1  // Report ID (0x01)
#define IMU_IDX_YAW_L    2  // Yaw low byte
#define IMU_IDX_YAW_H    3  // Yaw high byte
#define IMU_IDX_PITCH_L  4
#define IMU_IDX_PITCH_H  5
#define IMU_IDX_ROLL_L   6
#define IMU_IDX_ROLL_H   7
#define IMU_CHKSUM_FIRST 2   // First checksum byte index
#define IMU_CHKSUM_LAST  17  // Last checksum byte index
#define IMU_IDX_CHECKSUM 18  // Checksum byte index
#define IMU_HDR0         170 // Expected header (0xAA)

// IMU processing
#define IMU_RAW_TO_DEGREES_DIVISOR 100.0f

// ================= Angular Velocity Filtering ================= //
#define SENSORS_ANGULAR_VEL_FILTER_ALPHA 0.7f

// ================= Line Sensor Configuration ================= //
#define LINE_SENSOR_COUNT             8
#define LINE_SENSOR_I2C_ADDR          0x38
#define LINE_SENSOR_I2C_BAUD          100000
#define LINE_INTERSECTION_DEBOUNCE_MS 50

#endif // CONFIG_SENSORS_H
