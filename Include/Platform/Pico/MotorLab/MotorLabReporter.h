#ifndef MOTORLAB_REPORTER_H
#define MOTORLAB_REPORTER_H

#include <cstdint>

/**
 * @file MotorLabReporter.h
 * @brief Data logging and reporting for motor lab trials
 *
 * Provides interval-based data logging during motor characterization
 * trials. Outputs CSV-compatible data for analysis in spreadsheets
 * or Python scripts.
 *
 * Based on UKMARS motorlab by Peter Harrison.
 */

/**
 * @brief Data reporter for motor lab trials
 *
 * Manages timing and outputs telemetry data at configurable intervals.
 * Two reporting modes: profile data and controller data.
 */
class MotorLabReporter {
  public:
    /**
     * @brief Constructor with default reporting interval
     * @param interval_ms Minimum time between reports (milliseconds)
     */
    explicit MotorLabReporter(uint32_t interval_ms = 10);

    /**
     * @brief Set reporting interval
     * @param interval_ms Time between reports in milliseconds
     */
    void setInterval(uint32_t interval_ms);

    /**
     * @brief Start a new reporting session
     *
     * Resets the timer and sample counter for a new trial.
     */
    void begin();

    /**
     * @brief Check if it's time to report
     *
     * Call this in your control loop to determine if enough time
     * has passed for the next data point.
     *
     * @param current_time_ms Current system time in milliseconds
     * @return true if ready to report, false otherwise
     */
    bool isTimeToReport(uint32_t current_time_ms);

    /**
     * @brief Print header line for profile data
     *
     * Outputs CSV column headers for profile reporting mode.
     */
    void printProfileHeader();

    /**
     * @brief Report profile data
     *
     * Logs motion profile telemetry (position, velocity, motor output).
     *
     * @param time_ms Timestamp (milliseconds since trial start)
     * @param set_position Target position from profile
     * @param actual_position Measured position from encoder
     * @param set_speed Target velocity from profile
     * @param actual_speed Measured velocity from encoder
     * @param motor_volts Voltage applied to motors
     */
    void reportProfile(uint32_t time_ms, float set_position, float actual_position,
                       float set_speed, float actual_speed, float motor_volts);

    /**
     * @brief Print header line for controller data
     *
     * Outputs CSV column headers for controller reporting mode.
     */
    void printControllerHeader();

    /**
     * @brief Report controller data
     *
     * Logs control loop telemetry (errors, outputs, feedforward).
     *
     * @param time_ms Timestamp (milliseconds since trial start)
     * @param set_position Target position
     * @param actual_position Measured position
     * @param set_speed Target velocity
     * @param actual_speed Measured velocity
     * @param control_volts PD controller output voltage
     * @param ff_volts Feedforward voltage
     * @param total_volts Total voltage applied (control + ff)
     */
    void reportController(uint32_t time_ms, float set_position, float actual_position,
                          float set_speed, float actual_speed,
                          float control_volts, float ff_volts, float total_volts);

    /**
     * @brief Print header for open-loop voltage sweep
     */
    void printOpenLoopHeader();

    /**
     * @brief Report open-loop trial data
     *
     * Logs steady-state response during voltage sweep calibration.
     *
     * @param time_ms Timestamp
     * @param voltage Applied voltage
     * @param speed Measured steady-state speed
     */
    void reportOpenLoop(uint32_t time_ms, float voltage, float speed);

    /**
     * @brief Print header for step response trial
     */
    void printStepHeader();

    /**
     * @brief Report step response data
     *
     * Logs transient response during step input trial.
     *
     * @param time_ms Timestamp
     * @param step_voltage Step input voltage
     * @param speed Measured speed response
     * @param position Measured position response
     */
    void reportStep(uint32_t time_ms, float step_voltage, float speed, float position);

    /**
     * @brief Get number of samples reported in current session
     * @return Sample count
     */
    uint32_t getSampleCount() const { return sample_count_; }

  private:
    uint32_t interval_ms_;      // Minimum time between reports
    uint32_t last_report_ms_;   // Timestamp of last report
    uint32_t start_time_ms_;    // Session start time
    uint32_t sample_count_;     // Number of samples reported
};

#endif  // MOTORLAB_REPORTER_H
