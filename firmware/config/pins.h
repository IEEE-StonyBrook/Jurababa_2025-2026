/**
 * @file pins.h
 * @brief GPIO pin assignments for all hardware peripherals
 *
 * This file centralizes all pin definitions for the Pico micromouse.
 * Change pins here to rewire the robot without touching other code.
 */
#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

// ================= Motor Pins ================= //
// Left motor (TB6552FNG + hardware inverter)
#define PIN_MOTOR_L_DIR 2 // Left motor direction (CTRL → inverter → IN1/IN2)
#define PIN_MOTOR_L_PWM 3 // Left motor speed PWM

// Right motor (TB6552FNG + hardware inverter)
#define PIN_MOTOR_R_DIR 6 // Right motor direction (CTRL → inverter → IN1/IN2)
#define PIN_MOTOR_R_PWM 7 // Right motor speed PWM

// ================= Encoder Pins ================= //
// Quadrature encoders (PIO-driven)
#define PIN_ENCODER_L_A 8  // Left encoder channel A (OUT_A_1)
#define PIN_ENCODER_L_B 1  // Left encoder channel B (OUT_B_1)
#define PIN_ENCODER_R_A 10 // Right encoder channel A (OUT_A_2)
#define PIN_ENCODER_R_B 11 // Right encoder channel B (OUT_B_2)

// ================= IMU Pins ================= //
// BNO085 IMU (UART RVC mode)
#define PIN_IMU_RX 9 // IMU UART receive (TX from IMU)

// ================= ToF Sensor Pins ================= //
// VL53L0X Time-of-Flight sensors (I2C with XSHUT control)
#define PIN_TOF_LEFT_XSHUT  0 // Left ToF shutdown
#define PIN_TOF_FRONT_XSHUT 29 // Front ToF shutdown
#define PIN_TOF_RIGHT_XSHUT 15 // Right ToF shutdown

// ================= Battery Monitor Pins ================= //
#define PIN_BATTERY_ADC 28 // ADC0 for battery voltage divider

// ================= Bluetooth Pins ================= //
// HC-05/06 Bluetooth module (UART0)
#define PIN_BT_TX 12 // Bluetooth transmit
#define PIN_BT_RX 13 // Bluetooth receive

// ================= Line Sensor Pins (I2C0) ================= //
#define PIN_LINE_SDA 0 // I2C0 SDA
#define PIN_LINE_SCL 1 // I2C0 SCL

#endif // CONFIG_PINS_H
