/**
 * @file config.h
 * @brief Master configuration include - imports all config headers
 *
 * Include this single file to get all configuration constants.
 * Individual config files can also be included separately.
 */
#ifndef CONFIG_CONFIG_H
#define CONFIG_CONFIG_H

// Include all configuration modules
#include "config/pins.h"      // GPIO pin assignments
#include "config/geometry.h"  // Robot dimensions
#include "config/tuning.h"    // Feedforward and PID gains
#include "config/motion.h"    // Speeds and accelerations
#include "config/sensors.h"   // Sensor thresholds

// ================= Hardware Constants ================= //
// Motor PWM configuration
#define PWM_WRAP         999u    // PWM counter wrap (resolution)
#define MIN_DUTY_0_TO_1  0.225f  // Minimum duty to overcome friction
#define MAX_VOLTAGE      6.0f    // Maximum safe motor voltage

// Battery
#define DEFAULT_BATTERY_VOLTAGE  8.35f  // Nominal 2S LiPo voltage

// ================= Drivetrain Constants ================= //
#define DRIVETRAIN_MIN_DT            0.001f   // Minimum valid dt (s)
#define DRIVETRAIN_FF_DEADZONE_MMPS  10.0f    // Feedforward deadzone (mm/s)
#define DRIVETRAIN_MAX_VELOCITY_MMPS 1500.0f  // Velocity sanity check

// ================= Multicore Constants ================= //
#define CORE_SLEEP_MS  250  // Inter-core sync sleep time (ms)

#endif  // CONFIG_CONFIG_H
