#include "control/pid.h"

namespace
{
float clampAbs(float value, float max_abs)
{
    if (value > max_abs)
        return max_abs;
    if (value < -max_abs)
        return -max_abs;
    return value;
}
} // namespace

PID::PID(float kp, float kd, float loop_frequency_hz)
    : kp_(kp), kd_(kd), loop_frequency_hz_(loop_frequency_hz)
{
}

float PID::update(float setpoint, float measured_change, float steering_adjustment)
{
    const float dt_s = 1.0f / loop_frequency_hz_;
    m_error_ += setpoint * dt_s - measured_change + steering_adjustment * dt_s;
    float diff    = m_error_ - m_prev_error_;
    m_prev_error_ = m_error_;
    // KD * diff * loop_frequency mirrors motorlab/motors.h:147 — pre-bakes
    // the per-second derivative scaling at runtime instead of compile time.
    return clampAbs(kp_ * m_error_ + kd_ * diff * loop_frequency_hz_, output_limit_);
}

void PID::setGains(float kp, float kd)
{
    kp_ = kp;
    kd_ = kd;
}

void PID::setOutputLimit(float limit)
{
    output_limit_ = (limit < 0.0f) ? 0.0f : limit;
}

void PID::reset()
{
    m_error_      = 0.0f;
    m_prev_error_ = 0.0f;
}
