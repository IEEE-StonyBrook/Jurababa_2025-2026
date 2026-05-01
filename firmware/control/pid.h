#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "config/motion.h"

/**
 * @brief Position-error PD controller (mazerunner-core / motorlab style)
 *
 * Per-tick math (with dt = 1 / loop_frequency_hz_):
 *   m_error      += setpoint * dt - measured_change
 *                  + steering_adjustment * dt;
 *   diff          = m_error - m_prev_error;        // per-LOOP diff
 *   m_prev_error  = m_error;
 *   output        = kp * m_error + kd * diff * loop_frequency_hz_;
 *
 * The `* loop_frequency_hz_` on the D-term mirrors motorlab's
 * `KD * diff * LOOP_FREQUENCY` (motors.h:147). Each PID instance carries its
 * own loop rate so forward (encoder-paced 500 Hz) and rotation (IMU-paced
 * 100 Hz) controllers can coexist without one polluting the other's dt.
 *
 * `setpoint` is the COMMANDED velocity (mm/s for forward, deg/s for rotation).
 * `measured_change` is the per-tick position delta (mm or deg) actually moved.
 * `steering_adjustment` is an additive rate term (only used by the rotation
 * controller; sourced from sensor-driven wall centering). Defaults to 0.
 *
 * No integral term: position-error integration is the integral.
 * No derivative LPF: per-loop diff has no `1/dt` to amplify noise.
 * No deadband.
 */
class PID
{
  public:
    explicit PID(float kp = 0.0f, float kd = 0.0f, float loop_frequency_hz = LOOP_FREQUENCY_HZ);

    /**
     * @brief Single-tick PD update.
     */
    float update(float setpoint, float measured_change, float steering_adjustment = 0.0f);

    void setGains(float kp, float kd);

    /** @brief Symmetric output saturation (e.g. battery voltage). */
    void setOutputLimit(float limit);

    /** @brief Clears integrated error and previous-error history. */
    void reset();

    /** @brief Read current accumulated position error (diagnostics). */
    float error() const { return m_error_; }

  private:
    float kp_;
    float kd_;
    float loop_frequency_hz_;
    float m_error_      = 0.0f;
    float m_prev_error_ = 0.0f;
    float output_limit_ = 1e6f;
};

#endif
