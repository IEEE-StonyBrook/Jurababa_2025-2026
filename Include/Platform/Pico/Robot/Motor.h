#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>

#include "../../../Include/Platform/Pico/Config.h"
#include "../../../Include/Platform/Pico/Robot/Battery.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Motor
{
  public:
    // Constructor sets up motor pins, PWM slice, and direction inversion.
    Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Battery* battery, bool invertDirection = false);

    // Applies a duty cycle in range [-1.0, 1.0]. Positive = forward.
    void applyPWM(float dutyCycle);

    // Applies desired volts, automatically scaled to live battery voltage.
    void applyVoltage(float desiredVolts);

    // Immediately stops the motor.
    void stopMotor();

  private:
    // Configures motor pins for PWM functionality.
    void configureMotorPins();

    // Configures PWM slice and channels.
    void configureMotorPWM();

    int      gpioMotorPinOne;
    int      gpioMotorPinTwo;
    int      pwmSliceNumber;
    int      forwardChannel;
    int      backwardChannel;
    Battery* battery;
    bool     invertDirection;
};

#endif