#include "motor_lab/reporter.h"

#include <cstdio>

MotorLabReporter::MotorLabReporter(uint32_t interval_ms)
    : interval_ms_(interval_ms), last_report_ms_(0), start_time_ms_(0), sample_count_(0)
{
}

void MotorLabReporter::setInterval(uint32_t interval_ms)
{
    interval_ms_ = interval_ms;
}

void MotorLabReporter::begin()
{
    last_report_ms_ = 0;
    start_time_ms_  = 0;
    sample_count_   = 0;
}

bool MotorLabReporter::isTimeToReport(uint32_t current_time_ms)
{
    if (sample_count_ == 0)
    {
        // First sample - record start time
        start_time_ms_  = current_time_ms;
        last_report_ms_ = current_time_ms;
        return true;
    }

    if (current_time_ms - last_report_ms_ >= interval_ms_)
    {
        last_report_ms_ = current_time_ms;
        return true;
    }

    return false;
}

void MotorLabReporter::printProfileHeader()
{
    printf("time_ms,set_pos,actual_pos,set_speed,actual_speed,motor_volts\n");
}

void MotorLabReporter::reportProfile(uint32_t time_ms, float set_position, float actual_position,
                                     float set_speed, float actual_speed, float motor_volts)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f\n", static_cast<unsigned long>(elapsed), set_position,
           actual_position, set_speed, actual_speed, motor_volts);
    sample_count_++;
}

void MotorLabReporter::printControllerHeader()
{
    printf("time_ms,set_pos,actual_pos,set_speed,actual_speed,ctrl_v,ff_v,total_v\n");
}

void MotorLabReporter::reportController(uint32_t time_ms, float set_position, float actual_position,
                                        float set_speed, float actual_speed, float control_volts,
                                        float ff_volts, float total_volts)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n", static_cast<unsigned long>(elapsed),
           set_position, actual_position, set_speed, actual_speed, control_volts, ff_volts,
           total_volts);
    sample_count_++;
}

void MotorLabReporter::printOpenLoopHeader()
{
    printf("time_ms,voltage,speed\n");
}

void MotorLabReporter::reportOpenLoop(uint32_t time_ms, float voltage, float speed)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f\n", static_cast<unsigned long>(elapsed), voltage, speed);
    sample_count_++;
}

void MotorLabReporter::printStepHeader()
{
    printf("time_ms,step_voltage,speed,position\n");
}

void MotorLabReporter::reportStep(uint32_t time_ms, float step_voltage, float speed, float position)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f,%.2f\n", static_cast<unsigned long>(elapsed), step_voltage, speed,
           position);
    sample_count_++;
}
