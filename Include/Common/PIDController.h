#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cfloat>
#include <cmath>

/**
 * @brief PID Controller for closed-loop control systems
 *
 * Implements a discrete PID controller with integral windup protection,
 * derivative filtering, deadband, and output limiting. Suitable for
 * motor velocity control, position tracking, and other feedback loops.
 *
 * The controller calculates output = Kp*e + Ki*∫e + Kd*de/dt
 * where e is the error signal (setpoint - measurement).
 */
class PIDController
{
  public:
    /**
     * @brief Constructs PID controller with specified gains
     * @param proportional_gain Proportional gain (Kp)
     * @param integral_gain Integral gain (Ki)
     * @param derivative_gain Derivative gain (Kd)
     */
    PIDController(float proportional_gain = 0.0f, float integral_gain = 0.0f, float derivative_gain = 0.0f);

    /**
     * @brief Calculates PID output from error signal
     * @param error Control error (setpoint - measurement)
     * @param time_delta Time step in seconds since last update
     * @return Control output signal
     */
    float calculateOutput(float error, float time_delta);

    /**
     * @brief Calculates PID output from setpoint and measurement
     * @param setpoint Desired value
     * @param measurement Current measured value
     * @param time_delta Time step in seconds since last update
     * @return Control output signal
     */
    float calculateOutputFromSetpoint(float setpoint, float measurement, float time_delta);

    /**
     * @brief Updates PID gains at runtime
     * @param proportional_gain Proportional gain (Kp)
     * @param integral_gain Integral gain (Ki)
     * @param derivative_gain Derivative gain (Kd)
     */
    void setGains(float proportional_gain, float integral_gain, float derivative_gain);

    /**
     * @brief Sets error deadband threshold
     * @param deadband_threshold If |error| < threshold, output forced to 0
     */
    void setDeadband(float deadband_threshold);

    /**
     * @brief Sets integral accumulator limit to prevent windup
     * @param max_integral Maximum absolute value for integral term
     */
    void setIntegralLimit(float max_integral);

    /**
     * @brief Sets output saturation limit
     * @param max_output Maximum absolute output value
     */
    void setOutputLimit(float max_output);

    /**
     * @brief Sets derivative low-pass filter coefficient
     * @param filter_alpha Filter strength (0-0.999, higher = more smoothing)
     */
    void setDerivativeFilterAlpha(float filter_alpha);

    /**
     * @brief Resets controller internal state
     *
     * Clears integral accumulator, error history, and derivative filter.
     * Call when switching control modes or reinitializing system.
     */
    void reset();

  private:
    // PID gains
    float proportional_gain_;
    float integral_gain_;
    float derivative_gain_;

    // Integral accumulator and its clamp
    float integral_accumulator_ = 0.0f;
    float integral_limit_ = 1e6f;

    // Previous error for derivative computation
    float previous_error_ = 0.0f;
    bool has_previous_error_ = false; // Prevents first-step derivative spike

    // Error deadband threshold
    float deadband_threshold_ = 0.0f;

    // Filtered derivative state
    float filtered_derivative_ = 0.0f;
    float derivative_filter_alpha_ = 0.9f; // Closer to 1 = more smoothing
    float derivative_limit_ = 1e6f; // Hard clamp for noise spikes

    // Output saturation limit (FLT_MAX = no limit)
    float output_limit_ = FLT_MAX;

    /**
     * @brief Clamps value to symmetric range [-max_abs, +max_abs]
     * @param value Input value
     * @param max_abs Maximum absolute value
     * @return Clamped value
     */
    static float clampAbs(float value, float max_abs);
};

#endif
