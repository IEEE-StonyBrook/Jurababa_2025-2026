/**
 * @file config.h
 * @brief Master config — include this one file to get everything
 */
#ifndef CONFIG_CONFIG_H
#define CONFIG_CONFIG_H

#include "config/geometry.h" // Robot dimensions, encoder, maze
#include "config/motion.h"   // Speed/accel limits, tolerances
#include "config/pins.h"     // GPIO pin assignments
#include "config/sensors.h"  // Sensor hardware config
#include "config/tuning.h"   // Calibration: feedforward + PID gains

// ===================== Motor Hardware ===================== //
#define PWM_WRAP    999u // PWM counter wrap (10-bit resolution)
#define MAX_VOLTAGE 6.0f // Maximum safe motor voltage

// ===================== Battery ===================== //
#define DEFAULT_BATTERY_VOLTAGE 8.35f // Nominal 2S LiPo

// ===================== Drivetrain ===================== //
#define DRIVETRAIN_MIN_DT            0.001f  // Minimum valid dt (seconds)
#define DRIVETRAIN_FF_DEADZONE_MMPS  10.0f   // Below this speed, no feedforward
#define DRIVETRAIN_MAX_VELOCITY_MMPS 2500.0f // Sanity clamp on velocity

// ===================== Multicore ===================== //
#define CORE_SLEEP_MS 250 // Inter-core sync sleep (ms)

#endif // CONFIG_CONFIG_H
