#include "driver_lab/driver_lab.h"

#include "config/config.h"
#include "control/drivetrain.h"
#include "control/robot.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/line_sensor.h"
#include "drivers/motor.h"
#include "drivers/tof.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <vector>

// ============================================================================
// Version and identification
// ============================================================================
static const char* DRIVERLAB_VERSION = "DRIVERLAB v2.0 (Jurababa)";

// ============================================================================
// Constructor and Initialization
// ============================================================================

// Robot mode: direct motor/encoder + Robot access (for yaw/omega)
DriverLab::DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                     Encoder* right_encoder, Battery* battery, Robot* robot)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery), reporter_(10), input_index_(0),
      echo_enabled_(false), history_count_(0), history_write_idx_(0), history_nav_idx_(-1),
      robot_(robot), left_tof_(nullptr), front_tof_(nullptr), right_tof_(nullptr),
      line_sensor_(nullptr)
{
    clearInput();
    temp_buffer_[0] = '\0';
}

// Robot mode with ToF sensors: direct motor/encoder + Robot + ToF access
DriverLab::DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                     Encoder* right_encoder, Battery* battery, Robot* robot, ToF* left_tof,
                     ToF* front_tof, ToF* right_tof)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery), reporter_(10), input_index_(0),
      echo_enabled_(false), history_count_(0), history_write_idx_(0), history_nav_idx_(-1),
      robot_(robot), left_tof_(left_tof), front_tof_(front_tof), right_tof_(right_tof),
      line_sensor_(nullptr)
{
    clearInput();
    temp_buffer_[0] = '\0';
}

// Robot mode with LineSensor: direct motor/encoder + Robot + LineSensor access
DriverLab::DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                     Encoder* right_encoder, Battery* battery, Robot* robot,
                     LineSensor* line_sensor)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery), reporter_(10), input_index_(0),
      echo_enabled_(false), history_count_(0), history_write_idx_(0), history_nav_idx_(-1),
      robot_(robot), left_tof_(nullptr), front_tof_(nullptr), right_tof_(nullptr),
      line_sensor_(line_sensor)
{
    clearInput();
    temp_buffer_[0] = '\0';
}

void DriverLab::init()
{
    settings_.initDefaults();

    printf("\n%s\n", DRIVERLAB_VERSION);
    printf("Loop frequency: %.0f Hz\n", LOOP_FREQUENCY_HZ);
    printf("Using mm/s units (MM_PER_TICK = %.4f)\n", MM_PER_TICK);
    printf("Type '?' for help\n\n");
    printPrompt();
}

// ============================================================================
// Motor Control Methods
// ============================================================================

void DriverLab::stopMotors()
{
    left_motor_->stop();
    right_motor_->stop();
}

void DriverLab::setMotorVoltage(float volts)
{
    float battery_volts = batteryVoltage();
    left_motor_->applyVoltage(volts, battery_volts);
    right_motor_->applyVoltage(volts, battery_volts);
}

void DriverLab::setLeftMotorVoltage(float volts)
{
    float battery_volts = batteryVoltage();
    left_motor_->applyVoltage(volts, battery_volts);
    right_motor_->applyVoltage(0, battery_volts);
}

void DriverLab::setRightMotorVoltage(float volts)
{
    float battery_volts = batteryVoltage();
    left_motor_->applyVoltage(0, battery_volts);
    right_motor_->applyVoltage(volts, battery_volts);
}

float DriverLab::batteryVoltage() const
{
    if (battery_ != nullptr)
    {
        return battery_->voltage();
    }
    return DEFAULT_BATTERY_VOLTAGE;
}

// ============================================================================
// Test Routines
// ============================================================================

void DriverLab::runOpenLoopTrial(float max_voltage, float step_voltage, uint32_t settle_time_ms)
{
    printf("\n=== Open Loop Voltage Sweep (Stereo + Steering) ===\n");
    printf("Max voltage: %.2f V, Step: %.2f V, Settle time: %lu ms\n", max_voltage, step_voltage,
           static_cast<unsigned long>(settle_time_ms));
    printf("Battery: %.2f V\n", batteryVoltage());
    printf("Steering PD: kP=%.3f, kD=%.3f\n", settings_.turnKP, settings_.turnKD);

    // Countdown to allow USB disconnect
    for (int i = 3; i > 0; i--)
    {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }
    printf("GO!\n\n");

    reporter_.begin();
    reporter_.printOpenLoopStereoHeader();

    Drivetrain* dt = robot_->drivetrain();
    dt->reset();
    robot_->setRotationGains(settings_.turnKP, 0.0f, settings_.turnKD);
    robot_->resetHeadingControl();

    // Per-motor steady-state data for independent regression
    std::vector<float> left_voltages, left_speeds;
    std::vector<float> right_voltages, right_speeds;
    std::vector<float> combined_voltages, combined_speeds;
    std::vector<float> tm_samples;

    constexpr float  MIN_SPEED_THRESHOLD = 10.0f; // mm/s
    constexpr size_t SKIP_SAMPLES        = 10;    // 100ms at 100Hz transient skip
    constexpr size_t OUTPUT_EVERY        = 5;     // Output every 5th sample
    constexpr float  MAX_STEER_VOLTS     = 1.0f;  // Clamp steering correction

    // Sweep from 0 to max_voltage
    for (float voltage = step_voltage; voltage <= max_voltage + 0.01f; voltage += step_voltage)
    {
        // Reset heading control at each voltage step to avoid error buildup
        robot_->resetHeadingControl();

        // Per-step buffers for steady-state averaging
        std::vector<float> step_left_speeds, step_right_speeds;
        std::vector<float> step_left_volts, step_right_volts;

        uint32_t step_start   = to_ms_since_boot(get_absolute_time());
        size_t   sample_count = 0;

        while (to_ms_since_boot(get_absolute_time()) - step_start < settle_time_ms)
        {
            robot_->update(LOOP_INTERVAL_S);

            float left_speed  = dt->velocity(WheelSide::LEFT);
            float right_speed = dt->velocity(WheelSide::RIGHT);

            // Get steering correction from Robot's heading controller
            float steer_volts = robot_->headingCorrection(0.0f, LOOP_INTERVAL_S);
            if (steer_volts > MAX_STEER_VOLTS)
                steer_volts = MAX_STEER_VOLTS;
            if (steer_volts < -MAX_STEER_VOLTS)
                steer_volts = -MAX_STEER_VOLTS;

            // Apply differential voltage (sign convention matches runPositionControl:
            // left = forward - rotation, right = forward + rotation)
            float left_v  = voltage - steer_volts;
            float right_v = voltage + steer_volts;
            float batt    = batteryVoltage();
            left_motor_->applyVoltage(left_v, batt);
            right_motor_->applyVoltage(right_v, batt);

            // Record after transient settles
            if (sample_count >= SKIP_SAMPLES)
            {
                step_left_speeds.push_back(left_speed);
                step_right_speeds.push_back(right_speed);
                step_left_volts.push_back(left_v);
                step_right_volts.push_back(right_v);

                if ((sample_count - SKIP_SAMPLES) % OUTPUT_EVERY == 0)
                {
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    reporter_.reportOpenLoopStereo(now, voltage, left_v, right_v, left_speed,
                                                   right_speed, steer_volts, robot_->yaw());
                }
            }

            sample_count++;
            sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
        }

        // Average last 10 samples as steady-state
        size_t avg_count = std::min(step_left_speeds.size(), size_t(10));
        float  left_ss = 0.0f, right_ss = 0.0f;
        float  left_v_ss = 0.0f, right_v_ss = 0.0f;
        if (avg_count > 0)
        {
            for (size_t i = step_left_speeds.size() - avg_count; i < step_left_speeds.size(); i++)
            {
                left_ss += step_left_speeds[i];
                right_ss += step_right_speeds[i];
                left_v_ss += step_left_volts[i];
                right_v_ss += step_right_volts[i];
            }
            left_ss /= static_cast<float>(avg_count);
            right_ss /= static_cast<float>(avg_count);
            left_v_ss /= static_cast<float>(avg_count);
            right_v_ss /= static_cast<float>(avg_count);
        }

        // Store per-motor steady-state with effective voltages
        left_voltages.push_back(left_v_ss);
        left_speeds.push_back(left_ss);
        right_voltages.push_back(right_v_ss);
        right_speeds.push_back(right_ss);

        // Combined average for backward compatibility
        float avg_speed = (left_ss + right_ss) / 2.0f;
        combined_voltages.push_back(voltage);
        combined_speeds.push_back(avg_speed);

        // Tm estimation from combined speed
        if (avg_speed > MIN_SPEED_THRESHOLD && !step_left_speeds.empty())
        {
            float target = avg_speed * 0.632f;
            for (size_t i = 1; i < step_left_speeds.size(); i++)
            {
                float s      = (step_left_speeds[i] + step_right_speeds[i]) / 2.0f;
                float s_prev = (step_left_speeds[i - 1] + step_right_speeds[i - 1]) / 2.0f;
                if (s_prev >= target && s >= target)
                {
                    tm_samples.push_back((i - 1) * LOOP_INTERVAL_S);
                    break;
                }
            }
        }
    }

    stopMotors();

    // Linear regression: speed = kM * voltage + intercept, kS = -intercept / kM
    auto linRegress = [](const std::vector<float>& v, const std::vector<float>& s, float min_speed,
                         float& kM_out, float& kS_out, int& n_out)
    {
        float sum_v = 0, sum_s = 0, sum_vs = 0, sum_vv = 0;
        n_out = 0;
        for (size_t i = 0; i < v.size(); i++)
        {
            if (s[i] > min_speed)
            {
                sum_v += v[i];
                sum_s += s[i];
                sum_vs += v[i] * s[i];
                sum_vv += v[i] * v[i];
                n_out++;
            }
        }
        if (n_out < 2)
        {
            kM_out = 0.0f;
            kS_out = 0.0f;
            return;
        }
        float denom = n_out * sum_vv - sum_v * sum_v;
        if (std::fabs(denom) < 1e-12f)
        {
            kM_out = 0.0f;
            kS_out = 0.0f;
            return;
        }
        kM_out          = (n_out * sum_vs - sum_v * sum_s) / denom;
        float intercept = (sum_s - kM_out * sum_v) / n_out;
        kS_out          = (std::fabs(kM_out) > 1e-6f) ? (-intercept / kM_out) : 0.0f;
    };

    // Combined regression
    float kM_calc = 0, kS_calc = 0;
    int   n_combined = 0;
    linRegress(combined_voltages, combined_speeds, MIN_SPEED_THRESHOLD, kM_calc, kS_calc,
               n_combined);

    // Per-motor regression
    float kM_L = 0, kS_L = 0, kM_R = 0, kS_R = 0;
    int   n_left = 0, n_right = 0;
    linRegress(left_voltages, left_speeds, MIN_SPEED_THRESHOLD, kM_L, kS_L, n_left);
    linRegress(right_voltages, right_speeds, MIN_SPEED_THRESHOLD, kM_R, kS_R, n_right);

    // Average Tm across all moving steps
    float tm_calc = 0.0f;
    if (!tm_samples.empty())
    {
        for (float t : tm_samples)
            tm_calc += t;
        tm_calc /= static_cast<float>(tm_samples.size());
    }

    // Update settings with results
    settings_.kM   = kM_calc;
    settings_.kS   = kS_calc;
    settings_.tm   = tm_calc;
    settings_.kM_L = kM_L;
    settings_.kM_R = kM_R;
    settings_.kS_L = kS_L;
    settings_.kS_R = kS_R;
    settings_.recalculateDerived();

    printf("\n=== OL Results ===\n");
    printf("Combined: kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_calc, kS_calc, n_combined);
    printf("Left:     kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_L, kS_L, n_left);
    printf("Right:    kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_R, kS_R, n_right);
    printf("  -> kV = %.7f V/(mm/s)\n", settings_.kV);
    printf("  -> kA = %.7f V/(mm/s^2)\n", settings_.kA);
    printf("Yaw drift: %.2f deg\n\n", robot_->yaw());
    printPrompt();
}

void DriverLab::runStepTrial(float step_voltage, uint32_t duration_ms)
{
    printf("\n=== Step Response Trial ===\n");
    printf("Step voltage: %.2f V, Duration: %lu ms\n", step_voltage,
           static_cast<unsigned long>(duration_ms));
    printf("Battery: %.2f V\n", batteryVoltage());

    // Countdown to allow USB disconnect
    for (int i = 3; i > 0; i--)
    {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }
    printf("GO!\n\n");

    reporter_.begin();
    reporter_.printStepHeader();

    Drivetrain* dt = robot_->drivetrain();
    dt->reset();

    // Collect speed samples for Tm calculation
    static constexpr int MAX_STEP_SAMPLES = 2000;
    std::vector<float>   step_times(MAX_STEP_SAMPLES);
    std::vector<float>   step_speeds(MAX_STEP_SAMPLES);
    int                  n_samples = 0;

    // Apply step input
    setMotorVoltage(step_voltage);

    // Record transient response
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed    = 0;

    while (elapsed < duration_ms)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        elapsed      = now - start_time;

        dt->update(LOOP_INTERVAL_S);

        float speed = (dt->velocity(WheelSide::LEFT) + dt->velocity(WheelSide::RIGHT)) / 2.0f;
        float pos   = (dt->position(WheelSide::LEFT) + dt->position(WheelSide::RIGHT)) / 2.0f;

        if (reporter_.isTimeToReport(now))
        {
            reporter_.reportStep(now, step_voltage, speed, pos);

            // Store for Tm calculation
            if (n_samples < MAX_STEP_SAMPLES)
            {
                step_times[n_samples]  = elapsed * 0.001f; // ms → seconds
                step_speeds[n_samples] = speed;
                n_samples++;
            }
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    stopMotors();

    // === Calculate Tm from collected data ===
    if (n_samples < 10)
    {
        printf("\nNot enough samples to calculate Tm\n");
        printPrompt();
        return;
    }

    // Steady-state speed = average of last 20% of samples
    int   tail_start  = n_samples - n_samples / 5;
    float sum_final   = 0.0f;
    int   final_count = 0;
    for (int i = tail_start; i < n_samples; i++)
    {
        sum_final += step_speeds[i];
        final_count++;
    }
    float v_final = sum_final / final_count;

    // 63.2% threshold
    float threshold = v_final * 0.632f;

    // Find first sample that crosses threshold
    float tm_calculated = 0.0f;
    bool  found         = false;
    for (int i = 0; i < n_samples; i++)
    {
        if (step_speeds[i] >= threshold)
        {
            // Linear interpolation between this sample and previous for accuracy
            if (i > 0 && step_speeds[i - 1] < threshold)
            {
                float frac =
                    (threshold - step_speeds[i - 1]) / (step_speeds[i] - step_speeds[i - 1]);
                tm_calculated = step_times[i - 1] + frac * (step_times[i] - step_times[i - 1]);
            }
            else
            {
                tm_calculated = step_times[i];
            }
            found = true;
            break;
        }
    }

    // Print results
    printf("\n=== Step Response Results ===\n");
    printf("Steady-state speed: %.1f mm/s\n", v_final);
    printf("63.2%% threshold:    %.1f mm/s\n", threshold);

    if (found && tm_calculated > 0.001f && tm_calculated < 2.0f)
    {
        settings_.tm = tm_calculated;
        settings_.td = tm_calculated / 2.0f; // Standard starting point: Td = Tm/2
        settings_.recalculateDerived();
        printf("Tm = %.5f s  (motor time constant)\n", settings_.tm);
        printf("  -> kA = %.7f V/(mm/s^2)\n", settings_.kA);
        printf("  -> Td = %.5f s  (= Tm/2)\n", settings_.td);
        printf("  -> kP = %.5f, kD = %.5f  (recomputed)\n", settings_.kP, settings_.kD);
    }
    else
    {
        printf("Tm: could not determine (speed may not have settled)\n");
        if (!found)
            printf("  Speed never reached 63.2%% of final value\n");
        else
            printf("  Tm = %.5f s (out of valid range)\n", tm_calculated);
    }

    printf("Samples: %d\n\n", n_samples);
    printPrompt();
}

void DriverLab::runMoveTrial(float distance, float top_speed, float acceleration, int mode)
{
    printf("\n=== Move Trial ===\n");
    printf("Distance: %.1f mm, Speed: %.1f mm/s, Accel: %.1f mm/s^2\n", distance, top_speed,
           acceleration);
    printf("Mode: %d (%s)\n", mode, mode == 0 ? "FF only" : (mode == 1 ? "PD only" : "FF+PD"));
    printf("Battery: %.2f V\n", batteryVoltage());

    // Countdown
    for (int i = 3; i > 0; i--)
    {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }
    printf("GO!\n\n");

    // Sync DriverLab settings → Robot/Drivetrain
    robot_->setForwardGains(settings_.kP, 0.0f, settings_.kD);
    robot_->setFeedforward(WheelSide::LEFT, settings_.kV, settings_.kS, settings_.kA);
    robot_->setFeedforward(WheelSide::RIGHT, settings_.kV, settings_.kS, settings_.kA);

    // Set control mode: 0=FF only, 1=PD only, 2=FF+PD
    if (mode == 0)
        robot_->setControlMode(ControlMode::FeedforwardOnly);
    else if (mode == 1)
        robot_->setControlMode(ControlMode::FeedbackOnly);
    else
        robot_->setControlMode(ControlMode::Full);

    // Delegate motion to Robot
    Drivetrain* dt = robot_->drivetrain();
    dt->reset();
    robot_->moveDistance(distance, top_speed, acceleration);

    reporter_.begin();
    printf("time_ms,set_speed,actual_speed,error,left_v,right_v\n");

    // Diagnostics tracking
    float max_speed_error = 0.0f;
    float sum_speed_error = 0.0f;
    float max_pos_error   = 0.0f;
    float max_volts       = 0.0f;
    int   diag_count      = 0;

    // Poll loop — Robot does all control, DriverLab just logs
    while (!robot_->motionComplete())
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        robot_->updateControl(LOOP_INTERVAL_S);

        float set_speed = robot_->targetForwardVel();
        float actual_speed =
            (dt->velocity(WheelSide::LEFT) + dt->velocity(WheelSide::RIGHT)) / 2.0f;
        float pos_error = robot_->forwardError();
        float left_v    = robot_->lastLeftVolts();
        float right_v   = robot_->lastRightVolts();

        // Track diagnostics
        float speed_err = std::fabs(set_speed - actual_speed);
        sum_speed_error += speed_err;
        diag_count++;
        if (speed_err > max_speed_error)
            max_speed_error = speed_err;
        if (std::fabs(pos_error) > max_pos_error)
            max_pos_error = std::fabs(pos_error);
        float abs_volts = std::max(std::fabs(left_v), std::fabs(right_v));
        if (abs_volts > max_volts)
            max_volts = abs_volts;

        // CSV output
        if (reporter_.isTimeToReport(now))
        {
            uint32_t elapsed = now - reporter_.startTime();
            printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f\n", static_cast<unsigned long>(elapsed), set_speed,
                   actual_speed, pos_error, left_v, right_v);
            reporter_.incrementSampleCount();
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    stopMotors();
    robot_->setControlMode(ControlMode::Full); // restore default

    float actual_dist     = (dt->position(WheelSide::LEFT) + dt->position(WheelSide::RIGHT)) / 2.0f;
    float final_pos_error = distance - actual_dist;
    float avg_speed_error = (diag_count > 0) ? (sum_speed_error / diag_count) : 0.0f;

    printf("\n=== Move Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("\n--- Diagnostics ---\n");
    printf("  Final position error: %+.2f mm\n", final_pos_error);
    printf("  Max position error:   %.2f mm\n", max_pos_error);
    printf("  Max speed error:      %.1f mm/s\n", max_speed_error);
    printf("  Avg speed error:      %.1f mm/s\n", avg_speed_error);
    printf("  Max motor volts:      %.2f V\n", max_volts);
    printf("\n--- Tuning Tips ---\n");
    if (mode == 0 || mode == 2)
    {
        if (avg_speed_error > 20.0f)
            printf("  ! Steady-state error high: kV may be wrong (run OL to recalibrate)\n");
        if (max_speed_error > top_speed * 0.3f)
            printf("  ! Large transient error: increase kA or reduce acceleration\n");
    }
    if (mode == 1 || mode == 2)
    {
        if (max_volts > MAX_VOLTAGE * 0.8f)
            printf("  ! Near voltage saturation: reduce kP or lower speed/accel\n");
        if (max_pos_error > 10.0f)
            printf("  ! Position drift > 10mm: increase kP\n");
    }
    if (std::fabs(final_pos_error) > 5.0f)
        printf("  ! Stopping error > 5mm: check deceleration phase\n");
    if (max_speed_error < top_speed * 0.1f && std::fabs(final_pos_error) < 3.0f)
        printf("  Looks good! Speed tracking within 10%%, position error < 3mm.\n");
    printf("\n");
    printPrompt();
}

void DriverLab::runTurnTrial(float degrees, float top_omega, float alpha)
{
    printf("\n=== Turn Trial ===\n");
    printf("Angle: %.1f deg, Speed: %.1f deg/s, Accel: %.1f deg/s^2\n", degrees, top_omega, alpha);
    printf("TURN_KP: %.4f, TURN_KD: %.4f\n", settings_.turnKP, settings_.turnKD);
    printf("Battery: %.2f V\n", batteryVoltage());

    // Countdown
    for (int i = 3; i > 0; i--)
    {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }
    printf("GO!\n\n");

    // Sync DriverLab gains → Robot's controllers
    robot_->setRotationGains(settings_.turnKP, 0.0f, settings_.turnKD);
    robot_->setControlMode(ControlMode::Full);

    // Delegate motion to Robot
    robot_->resetYaw();
    robot_->turnInPlace(degrees, top_omega, alpha);

    reporter_.begin();
    printf("time_ms,set_omega,actual_yaw,actual_omega,error,left_v,right_v\n");

    // Diagnostics tracking
    float peak_overshoot_deg = 0.0f;
    float max_yaw_error      = 0.0f;
    float max_volts          = 0.0f;
    bool  past_target        = false;

    // Poll loop — Robot does all control, DriverLab just logs
    while (!robot_->motionComplete())
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        robot_->updateControl(LOOP_INTERVAL_S);

        float actual_yaw   = robot_->yaw();
        float actual_omega = robot_->omega();
        float rot_error    = robot_->rotationError();
        float left_v       = robot_->lastLeftVolts();
        float right_v      = robot_->lastRightVolts();

        // Track diagnostics
        float abs_error = std::fabs(rot_error);
        if (abs_error > max_yaw_error)
            max_yaw_error = abs_error;
        float abs_volts = std::max(std::fabs(left_v), std::fabs(right_v));
        if (abs_volts > max_volts)
            max_volts = abs_volts;

        // Detect overshoot
        if (std::fabs(actual_yaw) >= std::fabs(degrees) * 0.95f)
            past_target = true;
        if (past_target)
        {
            float overshoot = std::fabs(actual_yaw) - std::fabs(degrees);
            if (overshoot > peak_overshoot_deg)
                peak_overshoot_deg = overshoot;
        }

        // CSV output
        if (reporter_.isTimeToReport(now))
        {
            uint32_t elapsed = now - reporter_.startTime();
            printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n", static_cast<unsigned long>(elapsed),
                   robot_->targetAngularVel(), actual_yaw, actual_omega, rot_error, left_v,
                   right_v);
            reporter_.incrementSampleCount();
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    stopMotors();

    float final_error = degrees - robot_->yaw();

    printf("\n=== Turn Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("\n--- Diagnostics ---\n");
    printf("  Final error:     %+.2f deg\n", final_error);
    printf("  Peak overshoot:  %.2f deg (%.1f%%)\n", peak_overshoot_deg,
           (std::fabs(degrees) > 0.1f) ? (peak_overshoot_deg / std::fabs(degrees) * 100.0f) : 0.0f);
    printf("  Max yaw error:   %.2f deg\n", max_yaw_error);
    printf("  Max motor volts: %.2f V\n", max_volts);
    printf("\n--- Tuning Tips ---\n");
    if (peak_overshoot_deg > std::fabs(degrees) * 0.10f)
        printf("  ! Overshoot > 10%%: increase turnKD or decrease turnKP\n");
    if (std::fabs(final_error) > 3.0f)
        printf("  ! Final error > 3 deg: increase turnKP\n");
    if (max_volts > MAX_VOLTAGE * 0.9f)
        printf("  ! Near voltage saturation: reduce speed or lower turnKP\n");
    if (peak_overshoot_deg < 1.0f && std::fabs(final_error) < 2.0f &&
        max_volts < MAX_VOLTAGE * 0.7f)
        printf("  Looks good! Gains are well-tuned for this speed.\n");
    printf("\n");
    printPrompt();
}

// ============================================================================
// CLI Processing
// ============================================================================

bool DriverLab::processSerial()
{
    int result = readSerialLine();
    if (result == 1)
    {
        // Complete line received
        DriverLabArgs args = tokenize();
        if (args.argc > 0)
        {
            executeCommand(args);
        }
        clearInput();
        printPrompt();
        return true;
    }
    return false;
}

int DriverLab::readSerialLine()
{
    while (true)
    {
        // Check both USB and UART for input
        int  c         = getchar_timeout_us(0); // Check USB
        bool from_uart = false;

        // Also check UART directly (for Bluetooth input)
        if (c == PICO_ERROR_TIMEOUT && uart_is_readable(uart0))
        {
            c         = uart_getc(uart0);
            from_uart = true;
        }

        if (c == PICO_ERROR_TIMEOUT)
        {
            return 0; // No data available
        }

        char ch = static_cast<char>(c);

        // Handle newline (command complete)
        if (ch == '\n' || ch == '\r')
        {
            if (echo_enabled_)
            {
                printf("\n");
            }

            // Save non-empty command to history
            if (input_index_ > 0)
            {
                saveToHistory();
            }

            history_nav_idx_ = -1; // Reset navigation
            return 1;
        }

        // Handle backspace
        if (ch == '\b' || ch == 127)
        {
            if (input_index_ > 0)
            {
                input_index_--;
                input_buffer_[input_index_] = '\0';
                if (echo_enabled_)
                {
                    printf("\b \b");
                }
            }
            continue;
        }

        // Add printable characters to buffer
        if (isprint(ch))
        {
            ch = static_cast<char>(toupper(ch));
            if (echo_enabled_)
            {
                putchar(ch);
            }
            if (input_index_ < DRIVERLAB_INPUT_BUFFER_SIZE - 1)
            {
                input_buffer_[input_index_++] = ch;
                input_buffer_[input_index_]   = '\0';
            }
        }
    }
}

DriverLabArgs DriverLab::tokenize()
{
    DriverLabArgs args  = {0};
    char*         token = strtok(input_buffer_, " ,=");

    while (token != nullptr && args.argc < DRIVERLAB_MAX_ARGC)
    {
        args.argv[args.argc++] = token;
        token                  = strtok(nullptr, " ,=");
    }

    return args;
}

void DriverLab::executeCommand(const DriverLabArgs& args)
{
    const char* cmd = args.argv[0];

    // Help
    if (strcmp(cmd, "?") == 0 || strcmp(cmd, "HELP") == 0)
    {
        cmdHelp();
    }
    // Identification
    else if (strcmp(cmd, "ID") == 0)
    {
        cmdId();
    }
    // Settings
    else if (strcmp(cmd, "SETTINGS") == 0 || strcmp(cmd, "S") == 0)
    {
        cmdSettings();
    }
    else if (strcmp(cmd, "INIT") == 0)
    {
        cmdInitSettings();
    }
    // Motor model parameters
    else if (strcmp(cmd, "KM") == 0)
    {
        cmdSetKm(args);
    }
    else if (strcmp(cmd, "TM") == 0)
    {
        cmdSetTm(args);
    }
    // Controller parameters
    else if (strcmp(cmd, "ZETA") == 0)
    {
        cmdSetZeta(args);
    }
    else if (strcmp(cmd, "TD") == 0)
    {
        cmdSetTd(args);
    }
    else if (strcmp(cmd, "KP") == 0)
    {
        cmdSetKp(args);
    }
    else if (strcmp(cmd, "KD") == 0)
    {
        cmdSetKd(args);
    }
    // Feedforward parameters
    else if (strcmp(cmd, "BIAS") == 0)
    {
        cmdSetBiasFF(args);
    }
    else if (strcmp(cmd, "SPEEDFF") == 0)
    {
        cmdSetSpeedFF(args);
    }
    else if (strcmp(cmd, "ACCFF") == 0)
    {
        cmdSetAccFF(args);
    }
    // Hardware queries
    else if (strcmp(cmd, "BATTERY") == 0 || strcmp(cmd, "BAT") == 0)
    {
        cmdBattery();
    }
    else if (strcmp(cmd, "ENCODERS") == 0 || strcmp(cmd, "ENC") == 0)
    {
        cmdEncoders();
    }
    else if (strcmp(cmd, "YAW") == 0 || strcmp(cmd, "IMU") == 0)
    {
        cmdYaw();
    }
    else if (strcmp(cmd, "YAWVEL") == 0 || strcmp(cmd, "IMUVEL") == 0)
    {
        cmdYawVel();
    }
    else if (strcmp(cmd, "YAWCON") == 0 || strcmp(cmd, "IMUCON") == 0)
    {
        cmdYawContinuous(args);
    }
    else if (strcmp(cmd, "YAWRESET") == 0 || strcmp(cmd, "IMURESET") == 0)
    {
        cmdYawReset();
    }
    // ToF sensor queries
    else if (strcmp(cmd, "LTOF") == 0)
    {
        cmdLeftTof();
    }
    else if (strcmp(cmd, "FTOF") == 0)
    {
        cmdFrontTof();
    }
    else if (strcmp(cmd, "RTOF") == 0)
    {
        cmdRightTof();
    }
    else if (strcmp(cmd, "TOFCON") == 0)
    {
        cmdTofContinuous(args);
    }
    else if (strcmp(cmd, "LENC") == 0)
    {
        cmdLeftEncoder();
    }
    else if (strcmp(cmd, "RENC") == 0)
    {
        cmdRightEncoder();
    }
    else if (strcmp(cmd, "ENCRESET") == 0)
    {
        cmdEncoderReset();
    }
    else if (strcmp(cmd, "ENCCON") == 0)
    {
        cmdEncoderContinuous(args);
    }
    // Test commands
    else if (strcmp(cmd, "OPENLOOP") == 0 || strcmp(cmd, "OL") == 0)
    {
        cmdOpenLoop(args);
    }
    else if (strcmp(cmd, "STEP") == 0)
    {
        cmdStep(args);
    }
    else if (strcmp(cmd, "MOVE") == 0)
    {
        cmdMove(args);
    }
    else if (strcmp(cmd, "TURN") == 0)
    {
        cmdTurn(args);
    }
    else if (strcmp(cmd, "TURN_KP") == 0)
    {
        cmdSetTurnKp(args);
    }
    else if (strcmp(cmd, "TURN_KD") == 0)
    {
        cmdSetTurnKd(args);
    }
    else if (strcmp(cmd, "VOLTS") == 0 || strcmp(cmd, "V") == 0)
    {
        cmdVoltage(args);
    }
    else if (strcmp(cmd, "VL") == 0)
    {
        cmdVoltageLeft(args);
    }
    else if (strcmp(cmd, "VR") == 0)
    {
        cmdVoltageRight(args);
    }
    else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "X") == 0)
    {
        cmdStop();
    }
    // Export calibration to Config.h format
    else if (strcmp(cmd, "EXPORT") == 0)
    {
        cmdExport();
    }
    // GPIO diagnostic
    else if (strcmp(cmd, "GPIO") == 0)
    {
        cmdGpioDiag(args);
    }
    // Motor DIR pin test
    else if (strcmp(cmd, "DIRTEST") == 0)
    {
        cmdDirTest(args);
    }
    // Line sensor commands
    else if (strcmp(cmd, "LINPOS") == 0)
    {
        cmdLinePosition();
    }
    else if (strcmp(cmd, "LININTER") == 0)
    {
        cmdLineIntersection();
    }
    else if (strcmp(cmd, "LINCON") == 0)
    {
        cmdLineContinuous(args);
    }
    // Echo control
    else if (strcmp(cmd, "ECHO") == 0)
    {
        if (args.argc > 1 && strcmp(args.argv[1], "OFF") == 0)
        {
            echo_enabled_ = false;
            printf("Echo disabled\n");
        }
        else
        {
            echo_enabled_ = true;
            printf("Echo enabled\n");
        }
    }
    // History commands
    else if (strcmp(cmd, "R") == 0)
    {
        cmdRepeat();
    }
    else if (strcmp(cmd, "H") == 0)
    {
        cmdHistory();
    }
    else if (cmd[0] >= '1' && cmd[0] <= '9' && strlen(cmd) <= 2)
    {
        // Handle history selection (1-10)
        int selection = atoi(cmd);
        executeHistorySelection(selection);
    }
    else
    {
        printf("Unknown command: %s (type '?' for help)\n", cmd);
    }
}

void DriverLab::clearInput()
{
    input_index_     = 0;
    input_buffer_[0] = '\0';
}

void DriverLab::printPrompt()
{
    printf("> ");
}

bool DriverLab::parseFloat(const DriverLabArgs& args, int index, float min_val, float max_val,
                           float& result)
{
    if (index >= args.argc)
    {
        return false; // No value provided (will print current value)
    }

    float val = static_cast<float>(atof(args.argv[index]));
    if (val < min_val || val > max_val)
    {
        printf("Value out of range [%.4f, %.4f]\n", min_val, max_val);
        return false;
    }

    result = val;
    return true;
}

// ============================================================================
// CLI Command Implementations
// ============================================================================

void DriverLab::cmdHelp()
{
    printf("\n");
    printf("=== TRIALS (run in order) ===\n");
    printf("  OL   [max step settle_ms]      Voltage sweep    (default: 4 1 500)\n");
    printf("  STEP [volts duration_ms]        Step response    (default: 3 1000)\n");
    printf("  MOVE [mm mm/s mm/s^2 mode]      Forward motion   (default: 360 500 1000 0)\n");
    printf("       mode: 0=FF only, 1=PD only, 2=FF+PD\n");
    printf("  TURN [deg deg/s deg/s^2]        Turn in place    (default: 90 200 500)\n");
    printf("       +deg=right, -deg=left\n");
    printf("\n");
    printf("=== TUNING (set/get, omit val to read) ===\n");
    printf("  Motor:  KM [val]  TM [val]\n");
    printf("  FF:     BIAS [V]  SPEEDFF [val]  ACCFF [val]\n");
    printf("  Fwd PD: ZETA [val]  TD [val]  KP [val]  KD [val]\n");
    printf("  Rot PD: TURN_KP [val]  TURN_KD [val]\n");
    printf("\n");
    printf("=== SENSORS ===\n");
    printf("  BAT  ENC  LENC  RENC  ENCRESET\n");
    printf("  IMU  IMUVEL  IMURESET\n");
    printf("  LTOF  FTOF  RTOF\n");
    printf("  LINPOS  LININTER\n");
    printf("  Continuous: IMUCON/TOFCON/LINCON/ENCCON [ms] [interval]\n");
    printf("\n");
    printf("=== OTHER ===\n");
    printf("  V [volts]  X(stop)  GPIO  SETTINGS  INIT  EXPORT  ID\n");
    printf("\n");
}

void DriverLab::cmdId()
{
    printf("%s\n", DRIVERLAB_VERSION);
    printf("Loop: %.0f Hz (%.1f ms)\n", LOOP_FREQUENCY_HZ, LOOP_INTERVAL_S * 1000.0f);
    printf("Max voltage: %.1f V\n", MAX_VOLTAGE);
    printf("Encoder: %.4f mm/tick\n", MM_PER_TICK);
}

void DriverLab::cmdSettings()
{
    settings_.print();
}

void DriverLab::cmdInitSettings()
{
    settings_.initDefaults();
    printf("Settings reset to defaults\n");
}

void DriverLab::cmdSetKm(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 100.0f, 10000.0f, val))
    {
        settings_.kM = val;
        settings_.recalculateDerived();
        printf("kM = %.2f (derived updated)\n", settings_.kM);
    }
    else if (args.argc == 1)
    {
        printf("kM = %.2f\n", settings_.kM);
    }
}

void DriverLab::cmdSetTm(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.01f, 2.0f, val))
    {
        settings_.tm = val;
        settings_.td = val / 2.0f; // Default Td = Tm/2
        settings_.recalculateDerived();
        printf("Tm = %.5f (Td and derived updated)\n", settings_.tm);
    }
    else if (args.argc == 1)
    {
        printf("Tm = %.5f\n", settings_.tm);
    }
}

void DriverLab::cmdSetZeta(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.1f, 2.0f, val))
    {
        settings_.zeta = val;
        settings_.recalculateDerived();
        printf("zeta = %.5f (Kp, Kd updated)\n", settings_.zeta);
    }
    else if (args.argc == 1)
    {
        printf("zeta = %.5f\n", settings_.zeta);
    }
}

void DriverLab::cmdSetTd(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.001f, 1.0f, val))
    {
        settings_.td = val;
        settings_.recalculateDerived();
        printf("Td = %.5f (Kp, Kd updated)\n", settings_.td);
    }
    else if (args.argc == 1)
    {
        printf("Td = %.5f\n", settings_.td);
    }
}

void DriverLab::cmdSetKp(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.kP = val;
        printf("kP = %.7f (manual override)\n", settings_.kP);
    }
    else if (args.argc == 1)
    {
        printf("kP = %.7f\n", settings_.kP);
    }
}

void DriverLab::cmdSetKd(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.kD = val;
        printf("kD = %.7f (manual override)\n", settings_.kD);
    }
    else if (args.argc == 1)
    {
        printf("kD = %.7f\n", settings_.kD);
    }
}

void DriverLab::cmdSetBiasFF(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 3.0f, val))
    {
        settings_.kS = val;
        printf("kS = %.5f V\n", settings_.kS);
    }
    else if (args.argc == 1)
    {
        printf("kS = %.5f V\n", settings_.kS);
    }
}

void DriverLab::cmdSetSpeedFF(const DriverLabArgs& args)
{
    // Speed FF is typically derived from Km, but allow override
    float val;
    if (parseFloat(args, 1, 0.0f, 0.1f, val))
    {
        settings_.kV = val;
        printf("kV = %.7f V/(mm/s) (manual override)\n", settings_.kV);
    }
    else if (args.argc == 1)
    {
        printf("kV = %.7f V/(mm/s) [= 1/Km]\n", settings_.kV);
    }
}

void DriverLab::cmdSetAccFF(const DriverLabArgs& args)
{
    // Accel FF is typically derived from Km and Tm, but allow override
    float val;
    if (parseFloat(args, 1, 0.0f, 0.01f, val))
    {
        settings_.kA = val;
        printf("kA = %.7f V/(mm/s^2) (manual override)\n", settings_.kA);
    }
    else if (args.argc == 1)
    {
        printf("kA = %.7f V/(mm/s^2) [= Tm/Km]\n", settings_.kA);
    }
}

void DriverLab::cmdBattery()
{
    if (battery_ != nullptr)
    {
        printf("Battery: %.2f V (ADC: %u)\n", battery_->voltage(), battery_->rawADC());
    }
    else
    {
        printf("Battery monitor not available (default: %.2f V)\n", DEFAULT_BATTERY_VOLTAGE);
    }
}

void DriverLab::cmdEncoders()
{
    printf("Position:\n");
    if (left_encoder_)
    {
        float left_mm = left_encoder_->ticks() * MM_PER_TICK;
        printf("  Left:  %ld ticks = %.1f mm\n", static_cast<long>(left_encoder_->ticks()),
               left_mm);
    }
    else
    {
        printf("  Left:  N/A (not connected)\n");
    }
    if (right_encoder_)
    {
        float right_mm = right_encoder_->ticks() * MM_PER_TICK;
        printf("  Right: %ld ticks = %.1f mm\n", static_cast<long>(right_encoder_->ticks()),
               right_mm);
    }
    else
    {
        printf("  Right: N/A (not connected)\n");
    }

    Drivetrain* dt = robot_->drivetrain();
    printf("Velocity:\n");
    printf("  Avg: %.1f mm/s\n",
           (dt->velocity(WheelSide::LEFT) + dt->velocity(WheelSide::RIGHT)) / 2.0f);
    printf("  L: %.1f mm/s  R: %.1f mm/s\n", dt->velocity(WheelSide::LEFT),
           dt->velocity(WheelSide::RIGHT));
}

void DriverLab::cmdYaw()
{
    printf("Yaw: %.2f deg\n", robot_->yaw());
}

void DriverLab::cmdYawVel()
{
    printf("Angular velocity: %.2f deg/s\n", robot_->omega());
}

void DriverLab::cmdYawContinuous(const DriverLabArgs& args)
{
    uint32_t duration_ms = 5000;
    uint32_t interval_ms = 100;

    if (args.argc > 1)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[1]));
    if (args.argc > 2)
        interval_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    if (interval_ms < 10)
        interval_ms = 10;

    printf("\n=== Continuous Yaw (duration: %lu ms, interval: %lu ms) ===\n",
           static_cast<unsigned long>(duration_ms), static_cast<unsigned long>(interval_ms));
    printf("Time(ms)  Yaw(deg)  Omega(deg/s)\n");

    // Initial sleep to establish proper dt on first iteration
    // This prevents divide-by-near-zero causing huge omega spikes
    sleep_ms(interval_ms);

    uint32_t        start_time = to_ms_since_boot(get_absolute_time());
    absolute_time_t last_tick  = get_absolute_time();
    uint32_t        elapsed    = 0;

    while (elapsed < duration_ms)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;
        elapsed             = to_ms_since_boot(now) - start_time;

        // Keep robot sensor tracking (omega) up to date
        robot_->update(dt);

        printf("%7lu  %8.2f  %8.2f\n", static_cast<unsigned long>(elapsed), robot_->yaw(),
               robot_->omega());

        sleep_ms(interval_ms);
    }

    printf("=== Done ===\n");
}

void DriverLab::cmdYawReset()
{
    robot_->resetYaw();
    printf("Yaw and angular velocity reset to 0\n");
}

void DriverLab::cmdLeftTof()
{
    if (left_tof_ != nullptr)
    {
        printf("Left ToF: %.0f mm\n", left_tof_->distanceDirect());
    }
    else
    {
        printf("Left ToF not available\n");
    }
}

void DriverLab::cmdFrontTof()
{
    if (front_tof_ != nullptr)
    {
        printf("Front ToF: %.0f mm\n", front_tof_->distanceDirect());
    }
    else
    {
        printf("Front ToF not available\n");
    }
}

void DriverLab::cmdRightTof()
{
    if (right_tof_ != nullptr)
    {
        printf("Right ToF: %.0f mm\n", right_tof_->distanceDirect());
    }
    else
    {
        printf("Right ToF not available\n");
    }
}

void DriverLab::cmdTofContinuous(const DriverLabArgs& args)
{
    if (left_tof_ == nullptr && front_tof_ == nullptr && right_tof_ == nullptr)
    {
        printf("ToF sensors not available\n");
        return;
    }

    uint32_t duration_ms = 5000;
    uint32_t interval_ms = 100;

    if (args.argc > 1)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[1]));
    if (args.argc > 2)
        interval_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    if (interval_ms < 10)
        interval_ms = 10;

    printf("\n=== Continuous ToF (duration: %lu ms, interval: %lu ms) ===\n",
           static_cast<unsigned long>(duration_ms), static_cast<unsigned long>(interval_ms));
    printf("Time(ms)  Left(mm)  Front(mm)  Right(mm)\n");

    uint32_t        start_time = to_ms_since_boot(get_absolute_time());
    absolute_time_t last_tick  = get_absolute_time();
    uint32_t        elapsed    = 0;

    while (elapsed < duration_ms)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;
        elapsed             = to_ms_since_boot(now) - start_time;

        robot_->update(dt);

        float left_dist  = (left_tof_ != nullptr) ? left_tof_->distanceDirect() : 0.0f;
        float front_dist = (front_tof_ != nullptr) ? front_tof_->distanceDirect() : 0.0f;
        float right_dist = (right_tof_ != nullptr) ? right_tof_->distanceDirect() : 0.0f;

        printf("%7lu  %8.0f  %9.0f  %8.0f\n", static_cast<unsigned long>(elapsed), left_dist,
               front_dist, right_dist);

        sleep_ms(interval_ms);
    }

    printf("=== Done ===\n");
}

void DriverLab::cmdLeftEncoder()
{
    if (!left_encoder_)
    {
        printf("Left encoder not connected\n");
        return;
    }
    float left_mm = left_encoder_->ticks() * MM_PER_TICK;
    printf("Left: %.2f mm (%ld ticks)\n", left_mm, static_cast<long>(left_encoder_->ticks()));
}

void DriverLab::cmdRightEncoder()
{
    if (!right_encoder_)
    {
        printf("Right encoder not connected\n");
        return;
    }
    float right_mm = right_encoder_->ticks() * MM_PER_TICK;
    printf("Right: %.2f mm (%ld ticks)\n", right_mm, static_cast<long>(right_encoder_->ticks()));
}

void DriverLab::cmdEncoderReset()
{
    robot_->drivetrain()->reset();
    printf("Encoders reset to 0\n");
}

void DriverLab::cmdEncoderContinuous(const DriverLabArgs& args)
{
    uint32_t duration_ms = 5000;
    uint32_t interval_ms = 100;

    if (args.argc > 1)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[1]));
    if (args.argc > 2)
        interval_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    if (interval_ms < 10)
        interval_ms = 10;

    printf("\n=== Continuous Encoders (duration: %lu ms, interval: %lu ms) ===\n",
           static_cast<unsigned long>(duration_ms), static_cast<unsigned long>(interval_ms));
    printf("Time(ms)  Left(mm)  Right(mm)\n");

    uint32_t        start_time = to_ms_since_boot(get_absolute_time());
    absolute_time_t last_tick  = get_absolute_time();
    uint32_t        elapsed    = 0;

    while (elapsed < duration_ms)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;
        elapsed             = to_ms_since_boot(now) - start_time;

        robot_->drivetrain()->update(dt);

        float left_mm  = robot_->drivetrain()->position(WheelSide::LEFT);
        float right_mm = robot_->drivetrain()->position(WheelSide::RIGHT);

        printf("%7lu  %8.2f  %8.2f\n", static_cast<unsigned long>(elapsed), left_mm, right_mm);

        sleep_ms(interval_ms);
    }

    printf("=== Done ===\n");
}

void DriverLab::cmdOpenLoop(const DriverLabArgs& args)
{
    float    max_v     = 6.0f;
    float    step_v    = 0.5f;
    uint32_t settle_ms = 2000;

    if (args.argc > 1)
        max_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        step_v = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        settle_ms = static_cast<uint32_t>(atoi(args.argv[3]));

    runOpenLoopTrial(max_v, step_v, settle_ms);
}

void DriverLab::cmdStep(const DriverLabArgs& args)
{
    float    step_v      = 3.0f;
    uint32_t duration_ms = 1000;

    if (args.argc > 1)
        step_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    runStepTrial(step_v, duration_ms);
}

void DriverLab::cmdMove(const DriverLabArgs& args)
{
    // Default values in mm units (half cell = 90mm, typical micromouse speeds)
    float dist  = 480.0f; // mm (3 cell)
    float speed = 200.0f; // mm/s
    float accel = 500.0f; // mm/s^2
    int   mode  = 2;

    if (args.argc > 1)
        dist = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        speed = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        accel = static_cast<float>(atof(args.argv[3]));
    if (args.argc > 4)
        mode = atoi(args.argv[4]);

    runMoveTrial(dist, speed, accel, mode);
}

void DriverLab::cmdTurn(const DriverLabArgs& args)
{
    float degrees = 90.0f;
    float omega   = ROBOT_MAX_TURN_SPEED_DEGPS;
    float alpha   = ROBOT_BASE_ANGULAR_ACCEL_DEGPS2;

    if (args.argc > 1)
        degrees = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        omega = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        alpha = static_cast<float>(atof(args.argv[3]));

    runTurnTrial(degrees, omega, alpha);
}

void DriverLab::cmdSetTurnKp(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.turnKP = val;
        printf("TURN_KP = %.4f\n", settings_.turnKP);
    }
    else if (args.argc == 1)
    {
        printf("TURN_KP = %.4f\n", settings_.turnKP);
    }
}

void DriverLab::cmdSetTurnKd(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.turnKD = val;
        printf("TURN_KD = %.4f\n", settings_.turnKD);
    }
    else if (args.argc == 1)
    {
        printf("TURN_KD = %.4f\n", settings_.turnKD);
    }
}

void DriverLab::cmdVoltage(const DriverLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: V <voltage>\n");
        return;
    }

    float volts = static_cast<float>(atof(args.argv[1]));
    if (volts > MAX_VOLTAGE)
        volts = MAX_VOLTAGE;
    if (volts < -MAX_VOLTAGE)
        volts = -MAX_VOLTAGE;

    setMotorVoltage(volts);
    printf("Applied %.2f V to both motors\n", volts);
}

void DriverLab::cmdVoltageLeft(const DriverLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: VL <voltage>\n");
        return;
    }

    float volts = static_cast<float>(atof(args.argv[1]));
    if (volts > MAX_VOLTAGE)
        volts = MAX_VOLTAGE;
    if (volts < -MAX_VOLTAGE)
        volts = -MAX_VOLTAGE;

    setLeftMotorVoltage(volts);
    printf("Applied %.2f V to LEFT motor only\n", volts);
}

void DriverLab::cmdVoltageRight(const DriverLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: VR <voltage>\n");
        return;
    }

    float volts = static_cast<float>(atof(args.argv[1]));
    if (volts > MAX_VOLTAGE)
        volts = MAX_VOLTAGE;
    if (volts < -MAX_VOLTAGE)
        volts = -MAX_VOLTAGE;

    setRightMotorVoltage(volts);
    printf("Applied %.2f V to RIGHT motor only\n", volts);
}

void DriverLab::cmdStop()
{
    stopMotors();
    printf("Motors stopped\n");
}

void DriverLab::cmdExport()
{
    // Convert DriverLab calibration values to tuning.h format
    // DriverLab uses: mm/s, volts
    // tuning.h uses: mm/s, duty cycle

    float battery_volts = batteryVoltage();

    // Per-motor feedforward: duty = 1 / (kM * battery)
    float kvl_duty =
        (std::fabs(settings_.kM_L) > 1e-6f) ? (1.0f / (settings_.kM_L * battery_volts)) : 0.0f;
    float kvr_duty =
        (std::fabs(settings_.kM_R) > 1e-6f) ? (1.0f / (settings_.kM_R * battery_volts)) : 0.0f;
    float ksl_duty = settings_.kS_L / battery_volts;
    float ksr_duty = settings_.kS_R / battery_volts;

    // Acceleration feedforward from combined model
    float ka_duty = settings_.kA / battery_volts;

    printf("\n");
    printf("// ============================================================\n");
    printf("// DriverLab Export - Copy to tuning.h\n");
    printf("// Calibrated at battery voltage: %.2f V\n", battery_volts);
    printf("// ============================================================\n");
    printf("\n");
    printf("// Feedforward constants (duty-based, per-motor)\n");
    printf("#define FORWARD_KVL %.6ff  // Left velocity gain (duty per mm/s)\n", kvl_duty);
    printf("#define FORWARD_KVR %.6ff  // Right velocity gain (duty per mm/s)\n", kvr_duty);
    printf("#define FORWARD_KSL %.3ff     // Left static friction (duty)\n", ksl_duty);
    printf("#define FORWARD_KSR %.3ff     // Right static friction (duty)\n", ksr_duty);
    printf("\n");
    printf("// Acceleration feedforward (optional, set to 0 if not used)\n");
    printf("#define FORWARD_KAL %.7ff  // Left accel gain (duty per mm/s^2)\n", ka_duty);
    printf("#define FORWARD_KAR %.7ff  // Right accel gain (duty per mm/s^2)\n", ka_duty);
    printf("\n");
    printf("// Raw DriverLab values (for reference):\n");
    printf("//   kM_combined = %.2f mm/s/V\n", settings_.kM);
    printf("//   kM_L = %.2f mm/s/V, kM_R = %.2f mm/s/V\n", settings_.kM_L, settings_.kM_R);
    printf("//   kS_L = %.3f V, kS_R = %.3f V\n", settings_.kS_L, settings_.kS_R);
    printf("//   Tm = %.5f s\n", settings_.tm);
    printf("\n");
    printf("// Rotation PD gains\n");
    printf("#define ROT_KP %.4ff\n", settings_.turnKP);
    printf("#define ROT_KD %.4ff\n", settings_.turnKD);
    printf("// ============================================================\n");
    printf("\n");
}

void DriverLab::cmdGpioDiag(const DriverLabArgs& args)
{
    printf("\n=== Raw GPIO Diagnostic ===\n");
    printf("Reading GP%d (channel A) and GP%d (channel B)\n", PIN_ENCODER_R_A, PIN_ENCODER_R_B);
    printf("Slowly spin right wheel and watch for changes\n");
    printf("Both pins should toggle. If only one changes → wiring issue\n\n");

    // Set pins as inputs with pull-ups (match PIO config)
    gpio_init(PIN_ENCODER_R_A);
    gpio_set_dir(PIN_ENCODER_R_A, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_R_A);

    gpio_init(PIN_ENCODER_R_B);
    gpio_set_dir(PIN_ENCODER_R_B, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_R_B);

    printf("Sample  GP%d(A)  GP%d(B)  State\n", PIN_ENCODER_R_A, PIN_ENCODER_R_B);
    printf("------  ------  ------  -----\n");

    for (int i = 0; i < 100; i++)
    {
        bool pin_a = gpio_get(PIN_ENCODER_R_A);
        bool pin_b = gpio_get(PIN_ENCODER_R_B);
        int  state = (pin_a ? 2 : 0) | (pin_b ? 1 : 0); // 2-bit state (0-3)

        printf("%6d     %d       %d      %d\n", i, pin_a, pin_b, state);
        sleep_ms(100);
    }

    printf("\n=== Analysis ===\n");
    printf("Expected: Both columns toggle between 0 and 1 as wheel spins\n");
    printf("Expected: State cycles through 0→1→3→2→0 (or reverse)\n");
    printf("If only GP%d changes: Channel B (GP%d) wiring issue\n", PIN_ENCODER_R_A,
           PIN_ENCODER_R_B);
    printf("If only GP%d changes: Channel A (GP%d) wiring issue\n", PIN_ENCODER_R_B,
           PIN_ENCODER_R_A);
}

// ============================================================================
// Line Sensor Commands
// ============================================================================

void DriverLab::cmdLinePosition()
{
    if (line_sensor_ == nullptr)
    {
        printf("Line sensor not available\n");
        return;
    }

    line_sensor_->read();
    float position = line_sensor_->getPosition();
    bool  on_line  = line_sensor_->onLine();

    printf("Position: %.2f (%s)\n", position, on_line ? "on line" : "off line");
}

void DriverLab::cmdLineIntersection()
{
    if (line_sensor_ == nullptr)
    {
        printf("Line sensor not available\n");
        return;
    }

    line_sensor_->read();
    bool intersection = line_sensor_->detectIntersection();

    printf("Intersection: %s\n", intersection ? "YES" : "NO");
}

void DriverLab::cmdLineContinuous(const DriverLabArgs& args)
{
    if (line_sensor_ == nullptr)
    {
        printf("Line sensor not available\n");
        return;
    }

    uint32_t duration_ms = 5000;
    uint32_t interval_ms = 100;

    if (args.argc > 1)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[1]));
    if (args.argc > 2)
        interval_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    if (interval_ms < 10)
        interval_ms = 10;

    printf("\n=== Continuous Line Sensor (duration: %lu ms, interval: %lu ms) ===\n",
           static_cast<unsigned long>(duration_ms), static_cast<unsigned long>(interval_ms));
    printf("Time(ms)  Position  Intersect\n");

    uint32_t        start_time = to_ms_since_boot(get_absolute_time());
    absolute_time_t last_tick  = get_absolute_time();
    uint32_t        elapsed    = 0;

    while (elapsed < duration_ms)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;
        elapsed             = to_ms_since_boot(now) - start_time;

        // Keep IMU tracking active
        robot_->update(dt);

        // Read and display line sensor
        line_sensor_->read();
        float position     = line_sensor_->getPosition();
        bool  intersection = line_sensor_->detectIntersection();

        printf("%7lu  %8.2f  %9s\n", static_cast<unsigned long>(elapsed), position,
               intersection ? "YES" : "NO");

        sleep_ms(interval_ms);
    }

    printf("=== Done ===\n");
}

// ============================================================================
// Command History
// ============================================================================

void DriverLab::saveToHistory()
{
    // Don't save if same as most recent command
    if (history_count_ > 0)
    {
        int last_idx = (history_write_idx_ - 1 + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
        if (strcmp(input_buffer_, history_[last_idx]) == 0)
        {
            return;
        }
    }

    strncpy(history_[history_write_idx_], input_buffer_, DRIVERLAB_INPUT_BUFFER_SIZE - 1);
    history_[history_write_idx_][DRIVERLAB_INPUT_BUFFER_SIZE - 1] = '\0';

    history_write_idx_ = (history_write_idx_ + 1) % DRIVERLAB_HISTORY_SIZE;
    if (history_count_ < DRIVERLAB_HISTORY_SIZE)
    {
        history_count_++;
    }
}

void DriverLab::cmdRepeat()
{
    if (history_count_ == 0)
    {
        printf("No command history\n");
        return;
    }

    // Get most recent command
    int last_idx = (history_write_idx_ - 1 + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;

    // Copy to input buffer
    strncpy(input_buffer_, history_[last_idx], DRIVERLAB_INPUT_BUFFER_SIZE);
    printf("Repeating: %s\n", input_buffer_);

    // Parse and execute
    DriverLabArgs args = tokenize();
    if (args.argc > 0)
    {
        executeCommand(args);
    }
}

void DriverLab::cmdHistory()
{
    if (history_count_ == 0)
    {
        printf("No command history\n");
        return;
    }

    printf("Command History:\n");

    // Print from oldest to newest
    int oldest =
        (history_write_idx_ - history_count_ + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
    for (int i = 0; i < history_count_; i++)
    {
        int idx = (oldest + i) % DRIVERLAB_HISTORY_SIZE;
        printf("  %d: %s\n", i + 1, history_[idx]);
    }

    printf("Type number (1-%d) and press Enter to execute\n", history_count_);
}

void DriverLab::executeHistorySelection(int selection)
{
    if (selection < 1 || selection > history_count_)
    {
        printf("Invalid selection: %d (valid range: 1-%d)\n", selection, history_count_);
        return;
    }

    // Calculate index in circular buffer
    int oldest =
        (history_write_idx_ - history_count_ + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
    int idx = (oldest + selection - 1) % DRIVERLAB_HISTORY_SIZE;

    // Copy to input buffer
    strncpy(input_buffer_, history_[idx], DRIVERLAB_INPUT_BUFFER_SIZE);
    printf("Executing: %s\n", input_buffer_);

    // Parse and execute
    DriverLabArgs args = tokenize();
    if (args.argc > 0)
    {
        executeCommand(args);
    }
}

// ============================================================================
// Motor Direction Pin Diagnostic
// ============================================================================

void DriverLab::cmdDirTest(const DriverLabArgs& args)
{
    printf("\n=== Motor Direction Pin Test ===\n");
    printf("This applies constant 25%% PWM and toggles DIR pins\n");
    printf("Watch motors - they should alternate direction every 2 seconds\n");
    printf("Press any key to stop\n\n");

    // Drain input buffer
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
        ;
    while (uart_is_readable(uart0))
        uart_getc(uart0);

    // Get PWM slices/channels for direct control
    uint     left_slice    = pwm_gpio_to_slice_num(PIN_MOTOR_L_PWM);
    uint     left_channel  = pwm_gpio_to_channel(PIN_MOTOR_L_PWM);
    uint     right_slice   = pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM);
    uint     right_channel = pwm_gpio_to_channel(PIN_MOTOR_R_PWM);
    uint16_t pwm_level     = static_cast<uint16_t>(PWM_WRAP * 0.3f); // 30% duty

    int cycle = 0;
    while (true) // Run until interrupted
    {
        // Check for user input to stop
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT)
        {
            printf("\nStopped by user\n");
            break;
        }

        bool dir_state = (cycle % 2 == 0);

        printf("Cycle %d: DIR_L=%d, DIR_R=%d\n", cycle + 1, dir_state, dir_state);
        cycle++;

        // Set DIR pins FIRST
        gpio_put(PIN_MOTOR_L_DIR, dir_state);
        gpio_put(PIN_MOTOR_R_DIR, dir_state);

        // Then set PWM level directly (bypasses Motor class)
        pwm_set_chan_level(left_slice, left_channel, pwm_level);
        pwm_set_chan_level(right_slice, right_channel, pwm_level);

        sleep_ms(2000);
    }

    // Stop motors
    stopMotors();
    printf("=== Test Complete ===\n");
    printf("Expected: Both motors should have reversed direction\n");
    printf("If right motor didn't reverse: Hardware issue with GP2 or H-bridge\n");
}
