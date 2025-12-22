#ifndef CONFIG_H
#define CONFIG_H

// Control loop frequency and period.
#define LOOP_FREQUENCY_HZ 100.0f                     // Control updates per second.
#define LOOP_INTERVAL_S   (1.0f / LOOP_FREQUENCY_HZ) // Seconds per update.

// Encoder and wheel geometry.
#define WHEEL_DIAMETER_MM     39.5f   // Wheel diameter.
#define TICKS_PER_REVOLUTION  1400.0f // Encoder ticks per wheel turn.
#define MM_PER_TICK           ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION) // Distance per tick.
#define TO_CENTER_DISTANCE_MM ((167.5f - WHEEL_DIAMETER_MM) / 2.0f)

// Robot geometry.
#define WHEEL_BASE_MM 85.0f // Distance between left and right wheels.
#define DEG_PER_MM_DIFFERENCE                                                                      \
    (180.0f / (3.14159265f * WHEEL_BASE_MM)) // Deg per mm difference between wheels.

// Motor control and PWM.
#define PWM_WRAP        999u   // Max PWM counter (resolution).
#define MIN_DUTY_0_TO_1 0.225f // Minimum duty cycle to move motor.
#define MAX_VOLTAGE     6.0f   // Max safe motor voltage.

// Battery reference.
#define DEFAULT_BATTERY_VOLTAGE 8.35f // Nominal charged battery voltage.

// Feedforward constants.
#define FORWARD_KVL 0.000876f  // Left duty gain forward (duty per mm/s)
#define FORWARD_KVR 0.000850f  // Right duty gain forward (duty per mm/s)
#define FORWARD_KSL 0.1333f    // Left static friction duty forward (duty)
#define FORWARD_KSR 0.2038f    // Right static friction duty forward (duty)

#define REVERSE_KVL 0.000876f // TEMP: set after reverse sweep fit
#define REVERSE_KVR 0.000850f // TEMP: set after reverse sweep fit
#define REVERSE_KSL 0.1333f   // TEMP: set after reverse sweep fit
#define REVERSE_KSR 0.2038f   // TEMP: set after reverse sweep fit

// PID constants.
#define LEFT_WHEEL_KP   0.0075f
#define RIGHT_WHEEL_KP  0.0075f
#define LEFT_WHEEL_KI   0.00375f
#define RIGHT_WHEEL_KI  0.00375f
#define LEFT_WHEEL_KD   0.0f
#define RIGHT_WHEEL_KD  0.0f
#define YAW_KP          0.0f
#define YAW_KI          0.0f
#define YAW_KD          0.0f

// ================== Motion Constants ================== //
// Distances.
#define CELL_DISTANCE_MM      180.0f // One cell length in mm.
#define HALF_CELL_DISTANCE_MM 90.0f  // Half cell length in mm.

// Linear speed/acceleration.
#define FORWARD_TOP_SPEED   300.0f // Max forward speed (mm/s).
#define FORWARD_FINAL_SPEED 0.0f   // End speed for forward motions.
#define FORWARD_ACCEL       200.0f // Forward accel (mm/s^2).

// Rotational speed/acceleration.
#define TURN_TOP_SPEED   100.0f // Max angular speed (deg/s).
#define TURN_FINAL_SPEED 0.0f   // End angular speed.
#define TURN_ACCEL       50.0f  // Angular accel (deg/s^2).

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

// ToF Sensor Wall Distances
#define TOF_CELL_DEPTH_TO_CHECK_MM  40
#define TOF_LEFT_WALL_THRESHOLD_MM  100
#define TOF_RIGHT_WALL_THRESHOLD_MM 100
#define TOF_FRONT_WALL_THRESHOLD_MM 120

#define CORE_SLEEP_MS 250 // Sleep time for core loops
#endif