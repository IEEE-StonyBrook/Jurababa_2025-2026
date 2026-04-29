#include "control/pid.h"

#include "config/motion.h"

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

PID::PID(float kp, float kd) : kp_(kp), kd_(kd)
{
}

float PID::update(float setpoint, float measured_change, float steering_adjustment)
{
    m_error_ +=
        setpoint * LOOP_INTERVAL_S - measured_change + steering_adjustment * LOOP_INTERVAL_S;
    float diff    = m_error_ - m_prev_error_;
    m_prev_error_ = m_error_;
    return clampAbs(kp_ * m_error_ + kd_ * diff, output_limit_);
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
