#include "Common/PIDController.h"

PIDController::PIDController(float K_P, float K_I, float K_D) : K_P(K_P), K_I(K_I), K_D(K_D)
{
}

float PIDController::calculateOutputFromSetpoint(float target, float measurement, float dt)
{
    // Convert to error-based API
    return calculateOutput(target - measurement, dt);
}

float PIDController::calculateOutput(float error, float dt)
{
    // Protect against divide-by-zero or extremely small dt
    if (dt <= 1e-6f)
        dt = 1e-6f;

    // First-call handling so derivative starts at zero
    if (!hasLastError)
    {
        lastError    = error;
        hasLastError = true;
    }

    // Deadband: Treat small errors as zero (Prevents jitter)
    if (fabs(error) < deadband)
    {
        lastError = error;
        return 0.0f;
    }

    // Proportional term
    float pOutput = K_P * error;

    // Integral term with windup protection
    integralAccum += error * dt;
    integralAccum = clampAbs(integralAccum, integralMax);
    float iOutput = K_I * integralAccum;

    // Derivative term (Rate of change of error)
    float deriv = (error - lastError) / dt;
    lastError   = error;

    // Clamp + low-pass filter derivative to reduce noise
    deriv         = clampAbs(deriv, derivMax);
    dFiltered     = dAlpha * dFiltered + (1.0f - dAlpha) * deriv;
    float dOutput = K_D * dFiltered;

    // Sum PID terms and clamp final output
    return clampAbs(pOutput + iOutput + dOutput, outputMax);
}

void PIDController::setGains(float kp, float ki, float kd)
{
    K_P = kp;
    K_I = ki;
    K_D = kd;
}

void PIDController::setDeadband(float deadbandToReturnZero)
{
    deadband = (deadbandToReturnZero < 0.0f) ? 0.0f : deadbandToReturnZero;
}

void PIDController::setIntegralLimit(float integralMaxAbs)
{
    integralMax   = (integralMaxAbs < 0.0f) ? 0.0f : integralMaxAbs;
    integralAccum = clampAbs(integralAccum, integralMax);
}

void PIDController::setOutputLimit(float outputMaxAbs)
{
    outputMax = (outputMaxAbs < 0.0f) ? 0.0f : outputMaxAbs;
}

void PIDController::setDerivativeFilterAlpha(float alpha)
{
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 0.999f)
        alpha = 0.999f;
    dAlpha = alpha;
}

void PIDController::reset()
{
    // Reset all state that depends on history
    integralAccum = 0.0f;
    lastError     = 0.0f;
    hasLastError  = false;
    dFiltered     = 0.0f;
}

float PIDController::clampAbs(float value, float maxAbs)
{
    if (value > maxAbs)
        return maxAbs;
    if (value < -maxAbs)
        return -maxAbs;
    return value;
}