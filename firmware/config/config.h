/**
 * @file config.h
 * @brief Master config — include this one file to get everything
 */
#ifndef CONFIG_CONFIG_H
#define CONFIG_CONFIG_H

#include "config/boot.h"        // Boot mode/sensor selection and status LED
#include "config/geometry.h"    // Physical dimensions, encoder, maze
#include "config/motion.h"      // Speed/accel limits, tolerances
#include "config/pins.h"        // GPIO pin assignments
#include "config/sensors.h"     // Sensor hardware config
#include "config/smooth_turn.h" // UKMARS-style smooth turn parameters
#include "config/tuning.h"      // Calibration: feedforward + PID gains

// ===================== Motor Hardware ===================== //
#define PWM_WRAP    999u // PWM counter wrap (10-bit resolution)
#define MAX_VOLTAGE 6.0f // Maximum safe motor voltage

// ===================== Battery ===================== //
#define DEFAULT_BATTERY_VOLTAGE 8.35f // Nominal 2S LiPo

// Instantaneous rail voltage below which the pack is failing to deliver current.
// 2S LiPo at ~3.2 V/cell. Estimate — tune once real sag figures are measured.
#define BATTERY_SAG_WARN_V 6.4f
// Recovery margin, so a reading sitting on the threshold does not chatter.
#define BATTERY_SAG_CLEAR_V 0.2f

// ===================== Multicore ===================== //
#define CORE_SLEEP_MS 250 // Inter-core sync sleep (ms)

#endif // CONFIG_CONFIG_H
