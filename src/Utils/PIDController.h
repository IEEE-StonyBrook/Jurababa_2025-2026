#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cmath>

#include "pico/stdlib.h"
class PIDController {
 public:
  PIDController(float kP = 0.0f, float kI = 0.0f, float kD = 0.0f,
                float initialError = 0.0f, float integralMaxValue = 1000000.0f,
                float deadbandToReturnZero = 0.001f);
  ~PIDController();

  float calculateOutput(float newError);
  void setInitialError(float initialError);
  void setDeadband(float deadband);

 private:
  float kP, kI, kD;

  float accumulatedIntegral;
  float integralMaxValue;

  float previousCalculationError;

  float deadbandToReturnZero;

  absolute_time_t previousCalculationTime;
};
#endif