#ifndef CONTROL_PID_H
#define CONTROL_PID_H

/**
 * @brief Position-error PD controller (mazerunner-core style)
 *
 * Per-tick math:
 *   m_error      += setpoint * LOOP_INTERVAL_S - measured_change
 *                  + steering_adjustment * LOOP_INTERVAL_S;
 *   diff          = m_error - m_prev_error;        // per-LOOP diff
 *   m_prev_error  = m_error;
 *   output        = kp * m_error + kd * diff;
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
    PID(float kp = 0.0f, float kd = 0.0f);

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
    float m_error_      = 0.0f;
    float m_prev_error_ = 0.0f;
    float output_limit_ = 1e6f;
};

#endif
