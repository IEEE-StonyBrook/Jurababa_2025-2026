#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>

#include "Platform/Pico/Config.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

/**
 * @brief DC Motor driver using PWM control
 *
 * Provides high-level interface for controlling a DC motor via two GPIO pins
 * configured for PWM output. Supports bidirectional control with duty cycle
 * or voltage-based commands.
 */
class Motor
{
  public:
    /**
     * @brief Constructs motor controller with pin configuration
     * @param gpio_pin_one First motor control pin (PWM capable)
     * @param gpio_pin_two Second motor control pin (PWM capable)
     * @param invert_direction If true, swaps forward/backward directions
     */
    Motor(int gpio_pin_one, int gpio_pin_two, bool invert_direction = false);

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
     * @brief Immediately stops motor by setting both PWM channels to zero
     */
    void stopMotor();

  private:
    /**
     * @brief Configures GPIO pins for PWM function
     */
    void configureMotorPins();

    /**
     * @brief Initializes PWM slice and channel configuration
     */
    void configureMotorPWM();

    // GPIO pin assignments
    int gpio_pin_one_;
    int gpio_pin_two_;

    // PWM hardware configuration
    int pwm_slice_number_;
    int forward_channel_;
    int backward_channel_;

    // Motor direction inversion flag
    bool invert_direction_;
};

#endif