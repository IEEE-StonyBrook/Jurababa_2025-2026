#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include <cfloat>
#include <cmath>

/**
 * @brief PID Controller for closed-loop control systems
 *
 * Implements a discrete PID controller with integral windup protection,
 * derivative filtering, deadband, and output limiting. Suitable for
 * motor velocity control, position tracking, and other feedback loops.
 *
 * The controller calculates output = Kp*e + Ki*integral(e) + Kd*de/dt
 * where e is the error signal (setpoint - measurement).
 */
class PID
{
  public:
    /**
     * @brief Constructs PID controller with specified gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    /**
     * @brief Calculates PID output from error signal
     * @param error Control error (setpoint - measurement)
     * @param dt Time step in seconds since last update
     * @return Control output signal
     */
    float compute(float error, float dt);

    /**
     * @brief Calculates PID output from setpoint and measurement
     * @param setpoint Desired value
     * @param measurement Current measured value
     * @param dt Time step in seconds since last update
     * @return Control output signal
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief Updates PID gains at runtime
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Sets error deadband threshold
     * @param threshold If |error| < threshold, output forced to 0
     */
    void setDeadband(float threshold);

    /**
     * @brief Sets integral accumulator limit to prevent windup
     */
    void setIntegralLimit(float limit);

    /**
     * @brief Sets output saturation limit
     */
    void setOutputLimit(float limit);

    /**
     * @brief Sets derivative low-pass filter coefficient
     * @param alpha Filter strength (0-0.999, higher = more smoothing)
     */
    void setDerivativeFilterAlpha(float alpha);

    /**
     * @brief Resets controller internal state
     */
    void reset();

  private:
    float kp_;
    float ki_;
    float kd_;

    float integral_       = 0.0f;
    float integral_limit_ = 1e6f;

    float prev_error_     = 0.0f;
    bool  has_prev_error_ = false;

    float deadband_ = 0.0f;

    float filtered_derivative_ = 0.0f;
    float derivative_alpha_    = 0.9f;
    float derivative_limit_    = 1e6f;

    float output_limit_ = FLT_MAX;

    static float clampAbs(float value, float max_abs);
};

#endif
