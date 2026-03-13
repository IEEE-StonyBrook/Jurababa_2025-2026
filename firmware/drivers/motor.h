#ifndef DRIVERS_MOTOR_H
#define DRIVERS_MOTOR_H

#include <algorithm>
#include <cmath>

#include "config/config.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "common/log.h"

/**
 * @brief DC Motor driver using TB6552FNG with hardware inverter
 *
 * Each motor is controlled by two pins:
 *   - DIR pin (GPIO): sets direction via hardware inverter → IN1/IN2
 *   - PWM pin (hardware PWM): controls speed
 *
 * STBY is hardwired to 5V. Short brake occurs when PWM is low.
 */
class Motor
{
  public:
    /**
     * @brief Constructs motor controller with pin configuration
     * @param dir_pin GPIO pin for direction control (CTRL → inverter → IN1/IN2)
     * @param pwm_pin GPIO pin for PWM speed control
     * @param invert_direction If true, flips the direction logic
     */
    Motor(int dir_pin, int pwm_pin, bool invert_direction = false);

    /**
     * @brief Applies duty cycle command to motor
     * @param duty_cycle Duty cycle in range [-1.0, 1.0] (positive = forward)
     */
    void applyDuty(float duty_cycle);

    /**
     * @brief Applies voltage command to motor (scaled by battery voltage)
     * @param desired_volts Desired motor voltage
     * @param battery_volts Current battery voltage for scaling
     */
    void applyVoltage(float desired_volts, float battery_volts);

    /**
     * @brief Immediately stops motor (short brake via PWM low)
     */
    void stop();

  private:
    void configurePins();
    void configurePWM();

    int  dir_pin_;
    int  pwm_pin_;
    uint pwm_slice_;
    uint pwm_channel_;
    bool invert_direction_;
};

#endif
