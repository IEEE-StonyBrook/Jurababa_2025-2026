#include "../../Include/Common/PIDController.h"

PIDController::PIDController(float K_P, float K_I, float K_D, float initialError, float integralMax,
                             float deadbandToReturnZero)
    : K_P(K_P), K_I(K_I), K_D(K_D), lastError(initialError), integralMax(integralMax),
      deadband(deadbandToReturnZero)
{
    lastTime      = get_absolute_time();
    integralAccum = 0.0f;
}

float PIDController::calculateOutput(float error)
{
    float pOutput = K_P * error;
    // Return proportion if P-controller.
    if (K_I == 0.0f && K_D == 0.0f)
    {
        return pOutput;
    }

    // Calculate dt in seconds.
    absolute_time_t now = get_absolute_time();
    float           dt  = absolute_time_diff_us(lastTime, now) / 1e6f;
    lastTime            = now;
    // Eliminate extremely small dt.
    if (dt <= 1e-6f)
        dt = 1e-6f;

    // Calculate integral error in I-controller.
    integralAccum += error * dt;
    integralAccum = fmax(fmin(integralAccum, integralMax), -integralMax);
    float iOutput = K_I * integralAccum;

    // Calculate derivative error in D-controller.
    // FIXME: Add alpha filter if wiggle a lt at low speed.
    float dOutput = K_D * ((error - lastError) / dt);
    lastError     = error;

    // Sum all outputs in controller.
    float output = pOutput + iOutput + dOutput;

    // Return 0 for small outputs.
    return (fabs(output) < deadband) ? 0.0f : output;
}

void PIDController::setInitialError(float initialError)
{
    lastError = initialError;
}

void PIDController::setDeadband(float db)
{
    deadband = db;
}

void PIDController::reset()
{
    integralAccum = 0.0f;
    lastError     = 0.0f;
    lastTime      = get_absolute_time();
}