#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>

#include "../../../Common/LogSystem.h"
#include "../../../Common/PIDController.h"
#include "Battery.h"
#include "Encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Motor {
 public:
  Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Encoder* encoder,
        Battery* battery, bool invertMotorDirection = false);

  float getWheelPositionMM();
  float getWheelVelocityMMPerSec();

  void configurePIDWithFF(float K_P, float K_I, float K_D, float FF_KSF = 0.0f,
                          float FF_KVF = 0.0f, float FF_KSR = 0.0f,
                          float FF_KVR = 0.0f);

  void setDesiredVelocityMMPerSec(float velMMPerSec);
  void controlTick();
  void applyPWM(float duty);
  void stopMotor();

 private:
  void configureMotorPins();
  void configureMotorPWM();

  Encoder* encoder;
  Battery* battery;
  int gpioMotorPinOne, gpioMotorPinTwo;
  bool invertMotorDirection;

  int pwmSliceNumber;
  int fChannel, bChannel;

  float velMMPerSec{0.0f}, desiredVelMMPerSec{0.0f};

  absolute_time_t lastTime;
  float lastPosMM = 0.0f;
  float dtAccum = 0.0f;

  PIDController pidVelocityController;
  float FF_KSF{0.0f}, FF_KVF{0.0f}, FF_KSR{0.0f}, FF_KVR{0.0f};
};

#endif