#include "driver_lab/reporter.h"

#include <cstdio>

DriverLabReporter::DriverLabReporter(uint32_t interval_ms)
    : interval_ms_(interval_ms), last_report_ms_(0), start_time_ms_(0), sample_count_(0)
{
}

void DriverLabReporter::setInterval(uint32_t interval_ms)
{
    interval_ms_ = interval_ms;
}

void DriverLabReporter::begin()
{
    last_report_ms_ = 0;
    start_time_ms_  = 0;
    sample_count_   = 0;
}

bool DriverLabReporter::isTimeToReport(uint32_t current_time_ms)
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

void DriverLabReporter::printProfileHeader()
{
    printf("time_ms,set_pos,actual_pos,set_speed,actual_speed,motor_volts\n");
}

void DriverLabReporter::reportProfile(uint32_t time_ms, float set_position, float actual_position,
                                      float set_speed, float actual_speed, float motor_volts)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f\n", static_cast<unsigned long>(elapsed), set_position,
           actual_position, set_speed, actual_speed, motor_volts);
    sample_count_++;
}

void DriverLabReporter::printControllerHeader()
{
    printf("time_ms,set_pos,actual_pos,set_speed,actual_speed,ctrl_v,ff_v,total_v\n");
}

void DriverLabReporter::reportController(uint32_t time_ms, float set_position,
                                         float actual_position, float set_speed, float actual_speed,
                                         float control_volts, float ff_volts, float total_volts)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n", static_cast<unsigned long>(elapsed),
           set_position, actual_position, set_speed, actual_speed, control_volts, ff_volts,
           total_volts);
    sample_count_++;
}

void DriverLabReporter::printOpenLoopHeader()
{
    printf("time_ms,voltage,speed\n");
}

void DriverLabReporter::reportOpenLoop(uint32_t time_ms, float voltage, float speed)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f\n", static_cast<unsigned long>(elapsed), voltage, speed);
    sample_count_++;
}

void DriverLabReporter::printOpenLoopStereoHeader()
{
    printf("time_ms,cmd_v,left_v,right_v,left_speed,right_speed,yaw\n");
}

void DriverLabReporter::reportOpenLoopStereo(uint32_t time_ms, float cmd_voltage,
                                             float left_voltage, float right_voltage,
                                             float left_speed, float right_speed, float yaw_deg)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", static_cast<unsigned long>(elapsed), cmd_voltage,
           left_voltage, right_voltage, left_speed, right_speed, yaw_deg);
    sample_count_++;
}

void DriverLabReporter::printStepHeader()
{
    printf("time_ms,step_voltage,speed,position\n");
}

void DriverLabReporter::reportStep(uint32_t time_ms, float step_voltage, float speed,
                                   float position)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f,%.2f\n", static_cast<unsigned long>(elapsed), step_voltage, speed,
           position);
    sample_count_++;
}

void DriverLabReporter::printTurnOpenLoopHeader()
{
    printf("time_ms,diff_v,yaw,omega\n");
}

void DriverLabReporter::reportTurnOpenLoop(uint32_t time_ms, float diff_voltage, float yaw_deg,
                                           float omega_degps)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f,%.2f\n", static_cast<unsigned long>(elapsed), diff_voltage, yaw_deg,
           omega_degps);
    sample_count_++;
}

void DriverLabReporter::printTurnStepHeader()
{
    printf("time_ms,diff_v,yaw,omega\n");
}

void DriverLabReporter::reportTurnStep(uint32_t time_ms, float diff_voltage, float yaw_deg,
                                       float omega_degps)
{
    uint32_t elapsed = time_ms - start_time_ms_;
    printf("%lu,%.3f,%.2f,%.2f\n", static_cast<unsigned long>(elapsed), diff_voltage, yaw_deg,
           omega_degps);
    sample_count_++;
}
