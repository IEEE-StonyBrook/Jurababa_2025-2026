#include "Common/PIDController.h"

PIDController::PIDController(float proportional_gain, float integral_gain, float derivative_gain)
    : proportional_gain_(proportional_gain),
      integral_gain_(integral_gain),
      derivative_gain_(derivative_gain)
{
}

float PIDController::calculateOutputFromSetpoint(float setpoint, float measurement, float time_delta)
{
    return calculateOutput(setpoint - measurement, time_delta);
}

float PIDController::calculateOutput(float error, float time_delta)
{
    if (time_delta <= 1e-6f)
        time_delta = 1e-6f;

    if (!has_previous_error_)
    {
        previous_error_ = error;
        has_previous_error_ = true;
    }

    if (fabs(error) < deadband_threshold_)
    {
        previous_error_ = error;
        return 0.0f;
    }

    float proportional_output = proportional_gain_ * error;

    integral_accumulator_ += error * time_delta;
    integral_accumulator_ = clampAbs(integral_accumulator_, integral_limit_);
    float integral_output = integral_gain_ * integral_accumulator_;

    float derivative = (error - previous_error_) / time_delta;
    previous_error_ = error;

    // Low-pass filter derivative to reduce noise
    derivative = clampAbs(derivative, derivative_limit_);
    filtered_derivative_ = derivative_filter_alpha_ * filtered_derivative_ +
                          (1.0f - derivative_filter_alpha_) * derivative;
    float derivative_output = derivative_gain_ * filtered_derivative_;

    return clampAbs(proportional_output + integral_output + derivative_output, output_limit_);
}

void PIDController::setGains(float proportional_gain, float integral_gain, float derivative_gain)
{
    proportional_gain_ = proportional_gain;
    integral_gain_ = integral_gain;
    derivative_gain_ = derivative_gain;
}

void PIDController::setDeadband(float deadband_threshold)
{
    deadband_threshold_ = (deadband_threshold < 0.0f) ? 0.0f : deadband_threshold;
}

void PIDController::setIntegralLimit(float max_integral)
{
    integral_limit_ = (max_integral < 0.0f) ? 0.0f : max_integral;
    integral_accumulator_ = clampAbs(integral_accumulator_, integral_limit_);
}

void PIDController::setOutputLimit(float max_output)
{
    output_limit_ = (max_output < 0.0f) ? 0.0f : max_output;
}

void PIDController::setDerivativeFilterAlpha(float filter_alpha)
{
    if (filter_alpha < 0.0f)
        filter_alpha = 0.0f;
    if (filter_alpha > 0.999f)
        filter_alpha = 0.999f;
    derivative_filter_alpha_ = filter_alpha;
}

void PIDController::reset()
{
    integral_accumulator_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
    filtered_derivative_ = 0.0f;
}

float PIDController::clampAbs(float value, float max_abs)
{
    if (value > max_abs)
        return max_abs;
    if (value < -max_abs)
        return -max_abs;
    return value;
}