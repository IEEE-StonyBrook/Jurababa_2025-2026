#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cmath>

#include "pico/stdlib.h"
class PIDController
{
  public:
    PIDController(float K_P = 0.0f, float K_I = 0.0f, float K_D = 0.0f, float initialError = 0.0f,
                  float integralMax = 1000000.0f, float deadbandToReturnZero = 0.001f);

    float calculateOutput(float newError, float dt);
    void  setInitialError(float initialError);
    void  setDeadband(float deadband);
    void  reset();

  private:
    float K_P, K_I, K_D;

    float integralAccum;
    float integralMax;

    float lastError;

    float deadband;

    absolute_time_t lastTime;
};
#endif