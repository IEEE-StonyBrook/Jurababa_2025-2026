#include "Platform/Pico/MotorLab/MotorLab.h"
#include "Platform/Pico/Config.h"
#include "Platform/Pico/Robot/BatteryMonitor.h"
#include "Platform/Pico/Robot/Encoder.h"
#include "Platform/Pico/Robot/Motor.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ============================================================================
// Version and identification
// ============================================================================
static const char* MOTORLAB_VERSION = "MOTORLAB v1.0 (Jurababa)";

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Clamp voltage to safe operating range
 * @param volts Input voltage
 * @return Voltage clamped to [-MAX_VOLTAGE, MAX_VOLTAGE]
 */
static float clampVoltage(float volts)
{
    if (volts > MAX_VOLTAGE)
    {
        return MAX_VOLTAGE;
    }
    if (volts < -MAX_VOLTAGE)
    {
        return -MAX_VOLTAGE;
    }
    return volts;
}

// ============================================================================
// Constructor and Initialization
// ============================================================================

MotorLab::MotorLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                   Encoder* right_encoder, BatteryMonitor* battery_monitor)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_monitor_(battery_monitor),
      reporter_(10), // 10ms reporting interval
      input_index_(0), echo_enabled_(true), prev_left_ticks_(0), prev_right_ticks_(0),
      left_velocity_degps_(0.0f), right_velocity_degps_(0.0f)
{
    clearInput();
}

void MotorLab::init()
{
    settings_.initDefaults();

    // Override with values from Config.h where applicable
    settings_.deg_per_count = 360.0f / TICKS_PER_REVOLUTION;

    printf("\n%s\n", MOTORLAB_VERSION);
    printf("Loop frequency: %.0f Hz\n", LOOP_FREQUENCY_HZ);
    printf("Type '?' for help\n\n");
    printPrompt();
}

// ============================================================================
// Motor Control Methods
// ============================================================================

void MotorLab::stopMotors()
{
    left_motor_->stopMotor();
    right_motor_->stopMotor();
}

void MotorLab::setMotorVoltage(float volts)
{
    float battery_volts = getBatteryVoltage();
    left_motor_->applyVoltage(volts, battery_volts);
    right_motor_->applyVoltage(volts, battery_volts);
}

void MotorLab::setLeftMotorVoltage(float volts)
{
    float battery_volts = getBatteryVoltage();
    left_motor_->applyVoltage(volts, battery_volts);
    right_motor_->applyVoltage(0, battery_volts);
}

void MotorLab::setRightMotorVoltage(float volts)
{
    float battery_volts = getBatteryVoltage();
    left_motor_->applyVoltage(0, battery_volts);
    right_motor_->applyVoltage(volts, battery_volts);
}

float MotorLab::getBatteryVoltage() const
{
    if (battery_monitor_ != nullptr)
    {
        return battery_monitor_->getVoltage();
    }
    return DEFAULT_BATTERY_VOLTAGE;
}

float MotorLab::getEncoderPositionDeg() const
{
    int32_t left_ticks  = left_encoder_->getTickCount();
    int32_t right_ticks = right_encoder_->getTickCount();
    float   avg_ticks   = static_cast<float>(left_ticks + right_ticks) / 2.0f;
    return avg_ticks * settings_.deg_per_count;
}

float MotorLab::getEncoderVelocityDegps() const
{
    return (left_velocity_degps_ + right_velocity_degps_) / 2.0f;
}

void MotorLab::resetEncoders()
{
    left_encoder_->reset();
    right_encoder_->reset();
    prev_left_ticks_      = 0;
    prev_right_ticks_     = 0;
    left_velocity_degps_  = 0.0f;
    right_velocity_degps_ = 0.0f;
}

void MotorLab::updateEncoders(float dt)
{
    if (dt < 0.001f)
    {
        return; // Avoid division by zero
    }

    int32_t left_ticks  = left_encoder_->getTickCount();
    int32_t right_ticks = right_encoder_->getTickCount();

    int32_t delta_left  = left_ticks - prev_left_ticks_;
    int32_t delta_right = right_ticks - prev_right_ticks_;

    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Convert tick deltas to deg/s
    left_velocity_degps_  = (delta_left * settings_.deg_per_count) / dt;
    right_velocity_degps_ = (delta_right * settings_.deg_per_count) / dt;
}

// ============================================================================
// Test Routines
// ============================================================================

void MotorLab::runOpenLoopTrial(float max_voltage, float step_voltage, uint32_t settle_time_ms)
{
    printf("\n=== Open Loop Voltage Sweep ===\n");
    printf("Max voltage: %.2f V, Step: %.2f V, Settle time: %lu ms\n", max_voltage, step_voltage,
           static_cast<unsigned long>(settle_time_ms));
    printf("Battery: %.2f V\n\n", getBatteryVoltage());

    reporter_.begin();
    reporter_.printOpenLoopHeader();

    resetEncoders();

    // Sweep from 0 to max_voltage
    for (float voltage = step_voltage; voltage <= max_voltage + 0.01f; voltage += step_voltage)
    {
        // Apply voltage
        setMotorVoltage(voltage);

        // Wait for steady state
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        while (to_ms_since_boot(get_absolute_time()) - start_time < settle_time_ms)
        {
            updateEncoders(LOOP_INTERVAL_S);
            sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
        }

        // Measure steady-state speed (average over a few samples)
        float     speed_sum   = 0.0f;
        const int num_samples = 10;
        for (int i = 0; i < num_samples; i++)
        {
            updateEncoders(LOOP_INTERVAL_S);
            speed_sum += getEncoderVelocityDegps();
            sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
        }
        float avg_speed = speed_sum / static_cast<float>(num_samples);

        // Report data
        uint32_t now = to_ms_since_boot(get_absolute_time());
        reporter_.reportOpenLoop(now, voltage, avg_speed);
    }

    stopMotors();

    printf("\n=== Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.getSampleCount()));
    printf("To find Km: slope of speed vs voltage line\n");
    printf("To find bias_ff: x-intercept of the line\n\n");
    printPrompt();
}

void MotorLab::runStepTrial(float step_voltage, uint32_t duration_ms)
{
    printf("\n=== Step Response Trial ===\n");
    printf("Step voltage: %.2f V, Duration: %lu ms\n", step_voltage,
           static_cast<unsigned long>(duration_ms));
    printf("Battery: %.2f V\n\n", getBatteryVoltage());

    reporter_.begin();
    reporter_.printStepHeader();

    resetEncoders();

    // Apply step input
    setMotorVoltage(step_voltage);

    // Record transient response
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed    = 0;

    while (elapsed < duration_ms)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        elapsed      = now - start_time;

        updateEncoders(LOOP_INTERVAL_S);

        if (reporter_.isTimeToReport(now))
        {
            reporter_.reportStep(now, step_voltage, getEncoderVelocityDegps(),
                                 getEncoderPositionDeg());
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    stopMotors();

    printf("\n=== Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.getSampleCount()));
    printf("To find Tm: time to reach 63.2%% of final speed\n\n");
    printPrompt();
}

void MotorLab::runMoveTrial(float distance, float top_speed, float acceleration, int mode)
{
    const char* mode_name;
    switch (mode)
    {
        case 0:
            mode_name = "FF only";
            break;
        case 1:
            mode_name = "PD only";
            break;
        default:
            mode_name = "FF+PD";
            break;
    }

    printf("\n=== Move Trial ===\n");
    printf("Distance: %.1f deg, Speed: %.1f deg/s, Accel: %.1f deg/s^2\n", distance, top_speed,
           acceleration);
    printf("Mode: %d (%s)\n", mode, mode_name);
    printf("Battery: %.2f V\n\n", getBatteryVoltage());

    reporter_.begin();
    reporter_.printControllerHeader();

    resetEncoders();

    // Start motion profile
    profile_.start(distance, top_speed, acceleration, 0.0f);

    // Determine which control modes are active
    bool use_feedforward = (mode == 0 || mode == 2);
    bool use_controller  = (mode == 1 || mode == 2);

    // Initialize error accumulator for PD control
    float position_error = 0.0f;
    float prev_error     = 0.0f;

    // Control loop
    while (!profile_.isFinished())
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Update sensors
        updateEncoders(LOOP_INTERVAL_S);

        // Update profile
        profile_.update(LOOP_INTERVAL_S);

        // Get setpoints
        float set_position = profile_.getPosition();
        float set_speed    = profile_.getSpeed();
        float set_accel    = profile_.getAcceleration();

        // Get actual values
        float actual_position = getEncoderPositionDeg();
        float actual_speed    = getEncoderVelocityDegps();

        // Calculate control output
        float ff_volts      = 0.0f;
        float control_volts = 0.0f;

        // Feedforward (if enabled)
        if (use_feedforward)
        {
            ff_volts = settings_.bias_ff;
            if (set_speed > 0.0f)
            {
                ff_volts += settings_.speed_ff * set_speed;
            }
            else if (set_speed < 0.0f)
            {
                ff_volts = -settings_.bias_ff + settings_.speed_ff * set_speed;
            }
            ff_volts += settings_.acc_ff * set_accel;
        }

        // PD Controller (if enabled)
        if (use_controller)
        {
            // Incremental error accumulation (mazerunner-core style)
            float expected_delta = set_speed * LOOP_INTERVAL_S;
            float actual_delta   = actual_speed * LOOP_INTERVAL_S;
            position_error += (expected_delta - actual_delta);

            // PD output
            float derivative = (position_error - prev_error) / LOOP_INTERVAL_S;
            control_volts    = settings_.kp * position_error + settings_.kd * derivative;
            prev_error       = position_error;
        }

        // Total output (clamped to safe limits)
        float total_volts = clampVoltage(ff_volts + control_volts);

        // Apply to motors
        setMotorVoltage(total_volts);

        // Report data
        if (reporter_.isTimeToReport(now))
        {
            reporter_.reportController(now, set_position, actual_position, set_speed, actual_speed,
                                       control_volts, ff_volts, total_volts);
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    // Hold position briefly then stop
    sleep_ms(200);
    stopMotors();

    printf("\n=== Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.getSampleCount()));
    printf("Final position error: %.2f deg\n", profile_.getPosition() - getEncoderPositionDeg());
    printPrompt();
}

// ============================================================================
// CLI Processing
// ============================================================================

bool MotorLab::processSerial()
{
    int result = readSerialLine();
    if (result == 1)
    {
        // Complete line received
        MotorLabArgs args = tokenize();
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

int MotorLab::readSerialLine()
{
    while (true)
    {
        // Check both USB and UART for input
        int c = getchar_timeout_us(0);  // Check USB
        bool from_uart = false;

        // Also check UART directly (for Bluetooth input)
        if (c == PICO_ERROR_TIMEOUT && uart_is_readable(uart0))
        {
            c = uart_getc(uart0);
            from_uart = true;
            // Debug what we're receiving
            printf("<U:0x%02X>", c);
        }

        if (c == PICO_ERROR_TIMEOUT)
        {
            return 0; // No data available
        }

        char ch = static_cast<char>(c);

        // Debug all characters
        if (from_uart) {
            printf("[%c]", (ch >= 32 && ch < 127) ? ch : '?');
        }

        // Handle newline (command complete)
        if (ch == '\n' || ch == '\r')
        {
            if (echo_enabled_)
            {
                printf("\n");
            }
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
            if (input_index_ < MOTORLAB_INPUT_BUFFER_SIZE - 1)
            {
                input_buffer_[input_index_++] = ch;
                input_buffer_[input_index_]   = '\0';
            }
        }
    }
}

MotorLabArgs MotorLab::tokenize()
{
    MotorLabArgs args  = {0};
    char*        token = strtok(input_buffer_, " ,=");

    while (token != nullptr && args.argc < MOTORLAB_MAX_ARGC)
    {
        args.argv[args.argc++] = token;
        token                  = strtok(nullptr, " ,=");
    }

    return args;
}

/**
 * @brief Check if command matches any of the given names
 * @param cmd Command string to check
 * @param name1 Primary command name
 * @param name2 Optional alias (nullptr to skip)
 * @return true if command matches either name
 */
static bool cmdMatches(const char* cmd, const char* name1, const char* name2 = nullptr)
{
    if (strcmp(cmd, name1) == 0)
    {
        return true;
    }
    if (name2 != nullptr && strcmp(cmd, name2) == 0)
    {
        return true;
    }
    return false;
}

void MotorLab::executeCommand(const MotorLabArgs& args)
{
    const char* cmd = args.argv[0];

    // Help and system commands
    if (cmdMatches(cmd, "?", "HELP"))
    {
        cmdHelp();
    }
    else if (cmdMatches(cmd, "ID"))
    {
        cmdId();
    }
    else if (cmdMatches(cmd, "SETTINGS", "S"))
    {
        cmdSettings();
    }
    else if (cmdMatches(cmd, "INIT"))
    {
        cmdInitSettings();
    }
    // Motor model parameters
    else if (cmdMatches(cmd, "KM"))
    {
        cmdSetKm(args);
    }
    else if (cmdMatches(cmd, "TM"))
    {
        cmdSetTm(args);
    }
    // Controller parameters
    else if (cmdMatches(cmd, "ZETA"))
    {
        cmdSetZeta(args);
    }
    else if (cmdMatches(cmd, "TD"))
    {
        cmdSetTd(args);
    }
    else if (cmdMatches(cmd, "KP"))
    {
        cmdSetKp(args);
    }
    else if (cmdMatches(cmd, "KD"))
    {
        cmdSetKd(args);
    }
    // Feedforward parameters
    else if (cmdMatches(cmd, "BIAS"))
    {
        cmdSetBiasFF(args);
    }
    else if (cmdMatches(cmd, "SPEEDFF"))
    {
        cmdSetSpeedFF(args);
    }
    else if (cmdMatches(cmd, "ACCFF"))
    {
        cmdSetAccFF(args);
    }
    // Hardware queries
    else if (cmdMatches(cmd, "BATTERY", "BAT"))
    {
        cmdBattery();
    }
    else if (cmdMatches(cmd, "ENCODERS", "ENC"))
    {
        cmdEncoders();
    }
    // Test commands
    else if (cmdMatches(cmd, "OPENLOOP", "OL"))
    {
        cmdOpenLoop(args);
    }
    else if (cmdMatches(cmd, "STEP"))
    {
        cmdStep(args);
    }
    else if (cmdMatches(cmd, "MOVE"))
    {
        cmdMove(args);
    }
    else if (cmdMatches(cmd, "VOLTS", "V"))
    {
        cmdVoltage(args);
    }
    else if (cmdMatches(cmd, "VL"))
    {
        cmdVoltageLeft(args);
    }
    else if (cmdMatches(cmd, "VR"))
    {
        cmdVoltageRight(args);
    }
    else if (cmdMatches(cmd, "STOP", "X"))
    {
        cmdStop();
    }
    // Echo control
    else if (cmdMatches(cmd, "ECHO"))
    {
        echo_enabled_ = !(args.argc > 1 && strcmp(args.argv[1], "OFF") == 0);
        printf("Echo %s\n", echo_enabled_ ? "enabled" : "disabled");
    }
    else
    {
        printf("Unknown command: %s (type '?' for help)\n", cmd);
    }
}

void MotorLab::clearInput()
{
    input_index_     = 0;
    input_buffer_[0] = '\0';
}

void MotorLab::printPrompt()
{
    printf("> ");
}

bool MotorLab::parseFloat(const MotorLabArgs& args, int index, float min_val, float max_val,
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

void MotorLab::cmdHelp()
{
    printf("\n=== MotorLab Commands ===\n");
    printf("System:\n");
    printf("  ?          - Show this help\n");
    printf("  ID         - Show version info\n");
    printf("  SETTINGS   - Show all settings\n");
    printf("  INIT       - Reset to defaults\n");
    printf("\nMotor Model (set or get):\n");
    printf("  KM [val]   - Motor velocity constant (deg/s/V)\n");
    printf("  TM [val]   - Motor time constant (s)\n");
    printf("\nController (set or get):\n");
    printf("  ZETA [val] - Damping ratio\n");
    printf("  TD [val]   - Derivative time constant (s)\n");
    printf("  KP [val]   - Proportional gain\n");
    printf("  KD [val]   - Derivative gain\n");
    printf("\nFeedforward (set or get):\n");
    printf("  BIAS [val] - Static friction voltage\n");
    printf("  SPEEDFF    - Speed feedforward (auto from Km)\n");
    printf("  ACCFF      - Accel feedforward (auto from Km,Tm)\n");
    printf("\nHardware:\n");
    printf("  BAT        - Show battery voltage\n");
    printf("  ENC        - Show encoder values\n");
    printf("  V [volts]  - Apply voltage to motors\n");
    printf("  X          - Stop motors\n");
    printf("\nTests:\n");
    printf("  OL [max] [step] [settle_ms] - Open-loop sweep\n");
    printf("  STEP [volts] [duration_ms]  - Step response\n");
    printf("  MOVE [dist] [speed] [accel] [mode] - Move trial\n");
    printf("       mode: 0=FF, 1=PD, 2=FF+PD\n");
    printf("\n");
}

void MotorLab::cmdId()
{
    printf("%s\n", MOTORLAB_VERSION);
    printf("Loop: %.0f Hz (%.1f ms)\n", LOOP_FREQUENCY_HZ, LOOP_INTERVAL_S * 1000.0f);
    printf("Max voltage: %.1f V\n", MAX_VOLTAGE);
    printf("Encoder: %.4f deg/tick\n", settings_.deg_per_count);
}

void MotorLab::cmdSettings()
{
    settings_.print();
}

void MotorLab::cmdInitSettings()
{
    settings_.initDefaults();
    printf("Settings reset to defaults\n");
}

void MotorLab::cmdSetKm(const MotorLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 100.0f, 10000.0f, val))
    {
        settings_.km = val;
        settings_.recalculateDerived();
        printf("Km = %.2f (derived updated)\n", settings_.km);
    }
    else if (args.argc == 1)
    {
        printf("Km = %.2f\n", settings_.km);
    }
}

void MotorLab::cmdSetTm(const MotorLabArgs& args)
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

void MotorLab::cmdSetZeta(const MotorLabArgs& args)
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

void MotorLab::cmdSetTd(const MotorLabArgs& args)
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

void MotorLab::cmdSetKp(const MotorLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.kp = val;
        printf("Kp = %.7f (manual override)\n", settings_.kp);
    }
    else if (args.argc == 1)
    {
        printf("Kp = %.7f\n", settings_.kp);
    }
}

void MotorLab::cmdSetKd(const MotorLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 10.0f, val))
    {
        settings_.kd = val;
        printf("Kd = %.7f (manual override)\n", settings_.kd);
    }
    else if (args.argc == 1)
    {
        printf("Kd = %.7f\n", settings_.kd);
    }
}

void MotorLab::cmdSetBiasFF(const MotorLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 3.0f, val))
    {
        settings_.bias_ff = val;
        printf("bias_ff = %.5f V\n", settings_.bias_ff);
    }
    else if (args.argc == 1)
    {
        printf("bias_ff = %.5f V\n", settings_.bias_ff);
    }
}

void MotorLab::cmdSetSpeedFF(const MotorLabArgs& args)
{
    // Speed FF is typically derived from Km, but allow override
    float val;
    if (parseFloat(args, 1, 0.0f, 0.1f, val))
    {
        settings_.speed_ff = val;
        printf("speed_ff = %.7f V/(deg/s) (manual override)\n", settings_.speed_ff);
    }
    else if (args.argc == 1)
    {
        printf("speed_ff = %.7f V/(deg/s) [= 1/Km]\n", settings_.speed_ff);
    }
}

void MotorLab::cmdSetAccFF(const MotorLabArgs& args)
{
    // Accel FF is typically derived from Km and Tm, but allow override
    float val;
    if (parseFloat(args, 1, 0.0f, 0.01f, val))
    {
        settings_.acc_ff = val;
        printf("acc_ff = %.7f V/(deg/s^2) (manual override)\n", settings_.acc_ff);
    }
    else if (args.argc == 1)
    {
        printf("acc_ff = %.7f V/(deg/s^2) [= Tm/Km]\n", settings_.acc_ff);
    }
}

void MotorLab::cmdBattery()
{
    if (battery_monitor_ != nullptr)
    {
        printf("Battery: %.2f V (ADC: %u)\n", battery_monitor_->getVoltage(),
               battery_monitor_->getRawADC());
    }
    else
    {
        printf("Battery monitor not available (default: %.2f V)\n", DEFAULT_BATTERY_VOLTAGE);
    }
}

void MotorLab::cmdEncoders()
{
    printf("Left:  %ld ticks (%.2f deg)\n", static_cast<long>(left_encoder_->getTickCount()),
           left_encoder_->getTickCount() * settings_.deg_per_count);
    printf("Right: %ld ticks (%.2f deg)\n", static_cast<long>(right_encoder_->getTickCount()),
           right_encoder_->getTickCount() * settings_.deg_per_count);
    printf("Velocity: %.2f deg/s (L: %.2f, R: %.2f)\n", getEncoderVelocityDegps(),
           left_velocity_degps_, right_velocity_degps_);
}

void MotorLab::cmdOpenLoop(const MotorLabArgs& args)
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

void MotorLab::cmdStep(const MotorLabArgs& args)
{
    float    step_v      = 3.0f;
    uint32_t duration_ms = 1000;

    if (args.argc > 1)
        step_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[2]));

    runStepTrial(step_v, duration_ms);
}

void MotorLab::cmdMove(const MotorLabArgs& args)
{
    float dist  = 360.0f;
    float speed = 500.0f;
    float accel = 1000.0f;
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

void MotorLab::cmdVoltage(const MotorLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: V <voltage>\n");
        return;
    }

    float volts = clampVoltage(static_cast<float>(atof(args.argv[1])));
    setMotorVoltage(volts);
    printf("Applied %.2f V to both motors\n", volts);
}

void MotorLab::cmdVoltageLeft(const MotorLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: VL <voltage>\n");
        return;
    }

    float volts = clampVoltage(static_cast<float>(atof(args.argv[1])));
    setLeftMotorVoltage(volts);
    printf("Applied %.2f V to LEFT motor only\n", volts);
}

void MotorLab::cmdVoltageRight(const MotorLabArgs& args)
{
    if (args.argc < 2)
    {
        printf("Usage: VR <voltage>\n");
        return;
    }

    float volts = clampVoltage(static_cast<float>(atof(args.argv[1])));
    setRightMotorVoltage(volts);
    printf("Applied %.2f V to RIGHT motor only\n", volts);
}

void MotorLab::cmdStop()
{
    stopMotors();
    printf("Motors stopped\n");
}
