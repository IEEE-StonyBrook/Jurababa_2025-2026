#include "control/pid.h"

PID::PID(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
}

float PID::compute(float setpoint, float measurement, float dt)
{
    return compute(setpoint - measurement, dt);
}

float PID::compute(float error, float dt)
{
    if (dt <= 1e-6f)
        dt = 1e-6f;

    if (!has_prev_error_)
    {
        prev_error_ = error;
        has_prev_error_ = true;
    }

    if (fabs(error) < deadband_)
    {
        prev_error_ = error;
        return 0.0f;
    }

    float p_term = kp_ * error;

    integral_ += error * dt;
    integral_ = clampAbs(integral_, integral_limit_);
    float i_term = ki_ * integral_;

    float derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    derivative = clampAbs(derivative, derivative_limit_);
    filtered_derivative_ = derivative_alpha_ * filtered_derivative_ +
                          (1.0f - derivative_alpha_) * derivative;
    float d_term = kd_ * filtered_derivative_;

    return clampAbs(p_term + i_term + d_term, output_limit_);
}

void PID::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setDeadband(float threshold)
{
    deadband_ = (threshold < 0.0f) ? 0.0f : threshold;
}

void PID::setIntegralLimit(float limit)
{
    integral_limit_ = (limit < 0.0f) ? 0.0f : limit;
    integral_ = clampAbs(integral_, integral_limit_);
}

void PID::setOutputLimit(float limit)
{
    output_limit_ = (limit < 0.0f) ? 0.0f : limit;
}

void PID::setDerivativeFilterAlpha(float alpha)
{
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 0.999f)
        alpha = 0.999f;
    derivative_alpha_ = alpha;
}

void PID::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_error_ = false;
    filtered_derivative_ = 0.0f;
}

float PID::clampAbs(float value, float max_abs)
{
    if (value > max_abs)
        return max_abs;
    if (value < -max_abs)
        return -max_abs;
    return value;
}
