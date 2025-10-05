#ifndef CONFIG_H
#define CONFIG_H

// Control loop frequency and period.
#define LOOP_FREQUENCY_HZ 100.0f  // Control updates per second.
#define LOOP_INTERVAL_S (1.0f / LOOP_FREQUENCY_HZ)  // Seconds per update.

// Encoder and wheel geometry.
#define WHEEL_DIAMETER_MM 39.5f       // Wheel diameter.
#define TICKS_PER_REVOLUTION 1400.0f  // Encoder ticks per wheel turn.
#define MM_PER_TICK                    \
  ((WHEEL_DIAMETER_MM * 3.14159265f) / \
   TICKS_PER_REVOLUTION)  // Distance per tick.

// Robot geometry.
#define WHEEL_BASE_MM 86.8f  // Distance between left and right wheels.
#define DEG_PER_MM_DIFFERENCE \
  (180.0f /                   \
   (3.14159265f * WHEEL_BASE_MM))  // Deg per mm difference between wheels.

// Motor control and PWM.
#define PWM_WRAP 999u           // Max PWM counter (resolution).
#define MIN_DUTY_0_TO_1 0.225f  // Minimum duty cycle to move motor.
#define MAX_VOLTAGE 6.0f        // Max safe motor voltage.

// Battery reference.
#define DEFAULT_BATTERY_VOLTAGE 7.92f  // Nominal charged battery voltage.

// Feedforward constants.
#define SPEED_FFL 0.0075918f  // Left wheel speed gain forward (volts per mm/s).
#define SPEED_FFR \
  0.0081375f  // Right wheel speed gain forward (volts per mm/s).
#define BIAS_FFL \
  1.0049463f  // Left wheel static friction voltage forward (volts).
#define BIAS_FFR \
  1.1314120f  // Right wheel static friction voltage forward (volts).
#define ACC_FFL \
  0.0000000f  // Left wheel acceleration gain forward (volts per mm/s²).
#define ACC_FFR \
  0.0000000f  // Right wheel acceleration gain forward (volts per mm/s²).

// Feedbackward constants.
#define SPEED_FBL 0.0079290f  // Left wheel speed gain reverse (volts per mm/s).
#define SPEED_FBR \
  0.0073686f  // Right wheel speed gain reverse (volts per mm/s).
#define BIAS_FBL \
  0.4491618f  // Left wheel static friction voltage reverse (volts).
#define BIAS_FBR \
  0.9882377f  // Right wheel static friction voltage reverse (volts).
#define ACC_FBL \
  0.0000000f  // Left wheel acceleration gain reverse (volts per mm/s²).
#define ACC_FBR \
  0.0000000f  // Right wheel acceleration gain reverse (volts per mm/s²).

// Forward PD controller gains.
#define FWD_KP 0.00f  // Proportional gain for forward error.
#define FWD_KD 0.00f  // Derivative gain for forward error.

// Rotation PD controller gains.
#define ROT_KP 1.00f  // Proportional gain for rotation error.
#define ROT_KD 0.00f  // Derivative gain for rotation error.

// ================== Motion Constants ================== //
// Distances.
#define CELL_DISTANCE_MM 180.0f      // One cell length in mm.
#define HALF_CELL_DISTANCE_MM 90.0f  // Half cell length in mm.

// Linear speed/acceleration.
#define FORWARD_TOP_SPEED 150.0f  // Max forward speed (mm/s).
#define FORWARD_FINAL_SPEED 0.0f  // End speed for forward motions.
#define FORWARD_ACCEL 500.0f      // Forward accel (mm/s^2).

// Rotational speed/acceleration.
#define TURN_TOP_SPEED 180.0f  // Max angular speed (deg/s).
#define TURN_FINAL_SPEED 0.0f  // End angular speed.
#define TURN_ACCEL 720.0f      // Angular accel (deg/s^2).

#endif