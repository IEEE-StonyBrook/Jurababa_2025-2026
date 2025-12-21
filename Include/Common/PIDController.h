#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cfloat>
#include <cmath>

class PIDController
{
  public:
    // Create a PID with gains only (most common usage)
    PIDController(float K_P = 0.0f, float K_I = 0.0f, float K_D = 0.0f);

    // Main PID interface
    // Error = target - measurement
    // Dt    = timestep in seconds
    float calculateOutput(float error, float dt);

    // Convenience wrapper so callers don’t manually compute error
    float calculateOutputFromSetpoint(float target, float measurement, float dt);

    // Runtime tuning
    void setGains(float K_P, float K_I, float K_D);

    // If |error| < deadband, output is forced to 0
    void setDeadband(float deadbandToReturnZero);

    // Limits integral accumulation to prevent windup
    void setIntegralLimit(float integralMaxAbs);

    // Limits final output (e.g. motor duty clamp)
    void setOutputLimit(float outputMaxAbs);

    // Low-pass filter strength for derivative term (Noise control)
    void setDerivativeFilterAlpha(float alpha);

    // Clears internal state (Call when switching modes)
    void reset();

  private:
    // PID gains
    float K_P, K_I, K_D;

    // Integral accumulator and its clamp
    float integralAccum = 0.0f;
    float integralMax   = 1e6f;

    // Previous error for derivative computation
    float lastError    = 0.0f;
    bool  hasLastError = false; // Prevents first-step derivative spike

    // Error deadband
    float deadband = 0.0f;

    // Filtered derivative state
    float dFiltered = 0.0f;
    float dAlpha    = 0.9f; // Closer to 1 = more smoothing
    float derivMax  = 1e6f; // Hard clamp for noise spikes

    // Clamp for final output (FLT_MAX = no clamp)
    float outputMax = FLT_MAX;

    // Utility: Clamp value to ±maxAbs
    static float clampAbs(float value, float maxAbs);
};

#endif
