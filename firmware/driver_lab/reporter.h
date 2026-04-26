#ifndef DRIVER_LAB_REPORTER_H
#define DRIVER_LAB_REPORTER_H

#include <cstdint>

/**
 * @brief Data logging and reporting for motor lab trials
 *
 * Outputs CSV-compatible data for analysis in spreadsheets or Python.
 */
class DriverLabReporter
{
  public:
    explicit DriverLabReporter(uint32_t interval_ms = 10);

    void setInterval(uint32_t interval_ms);
    void begin();
    bool isTimeToReport(uint32_t current_time_ms);

    void printProfileHeader();
    void reportProfile(uint32_t time_ms, float set_position, float actual_position, float set_speed,
                       float actual_speed, float motor_volts);

    void printControllerHeader();
    void reportController(uint32_t time_ms, float set_position, float actual_position,
                          float set_speed, float actual_speed, float control_volts, float ff_volts,
                          float total_volts);

    void printOpenLoopHeader();
    void reportOpenLoop(uint32_t time_ms, float voltage, float speed);

    void printOpenLoopStereoHeader();
    void reportOpenLoopStereo(uint32_t time_ms, float cmd_voltage, float left_voltage,
                              float right_voltage, float left_speed, float right_speed,
                              float steer_volts);

    void printStepHeader();
    void reportStep(uint32_t time_ms, float step_voltage, float speed, float position);

    uint32_t sampleCount() const { return sample_count_; }
    uint32_t startTime() const { return start_time_ms_; }
    void     incrementSampleCount() { sample_count_++; }

  private:
    uint32_t interval_ms_;
    uint32_t last_report_ms_;
    uint32_t start_time_ms_;
    uint32_t sample_count_;
};

#endif
