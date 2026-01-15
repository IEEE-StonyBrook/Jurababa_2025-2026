#include "motor_lab/motor_lab.h"

#include "config/config.h"
#include "control/drivetrain.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"

#include "hardware/uart.h"
#include "pico/stdlib.h"

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
// Constructor and Initialization
// ============================================================================

// Standalone mode: direct motor/encoder access
MotorLab::MotorLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                   Encoder* right_encoder, Battery* battery)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery), drivetrain_(nullptr), reporter_(10),
      input_index_(0), echo_enabled_(true), prev_left_ticks_(0), prev_right_ticks_(0),
      left_velocity_mmps_(0.0f), right_velocity_mmps_(0.0f)
{
    clearInput();
}

// Integrated mode: use Drivetrain for velocity and motor control
MotorLab::MotorLab(Drivetrain* drivetrain, Encoder* left_encoder, Encoder* right_encoder,
                   Battery* battery)
    : left_motor_(nullptr), right_motor_(nullptr), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery), drivetrain_(drivetrain), reporter_(10),
      input_index_(0), echo_enabled_(true), prev_left_ticks_(0), prev_right_ticks_(0),
      left_velocity_mmps_(0.0f), right_velocity_mmps_(0.0f)
{
    clearInput();
}

void MotorLab::init()
{
    settings_.initDefaults();

    printf("\n%s\n", MOTORLAB_VERSION);
    printf("Loop frequency: %.0f Hz\n", LOOP_FREQUENCY_HZ);
    printf("Using mm/s units (MM_PER_TICK = %.4f)\n", MM_PER_TICK);
    printf("Type '?' for help\n\n");
    printPrompt();
}

// ============================================================================
// Motor Control Methods
// ============================================================================

void MotorLab::stopMotors()
{
    if (drivetrain_ != nullptr)
    {
        drivetrain_->stop();
    }
    else
    {
        left_motor_->stop();
        right_motor_->stop();
    }
}

void MotorLab::setMotorVoltage(float volts)
{
    if (drivetrain_ != nullptr)
    {
        drivetrain_->setVoltage(volts, volts);
    }
    else
    {
        float battery_volts = batteryVoltage();
        left_motor_->applyVoltage(volts, battery_volts);
        right_motor_->applyVoltage(volts, battery_volts);
    }
}

void MotorLab::setLeftMotorVoltage(float volts)
{
    if (drivetrain_ != nullptr)
    {
        drivetrain_->setVoltage(volts, 0.0f);
    }
    else
    {
        float battery_volts = batteryVoltage();
        left_motor_->applyVoltage(volts, battery_volts);
        right_motor_->applyVoltage(0, battery_volts);
    }
}

void MotorLab::setRightMotorVoltage(float volts)
{
    if (drivetrain_ != nullptr)
    {
        drivetrain_->setVoltage(0.0f, volts);
    }
    else
    {
        float battery_volts = batteryVoltage();
        left_motor_->applyVoltage(0, battery_volts);
        right_motor_->applyVoltage(volts, battery_volts);
    }
}

float MotorLab::batteryVoltage() const
{
    if (drivetrain_ != nullptr)
    {
        return drivetrain_->batteryVoltage();
    }
    if (battery_ != nullptr)
    {
        return battery_->voltage();
    }
    return DEFAULT_BATTERY_VOLTAGE;
}

float MotorLab::encoderPositionMM() const
{
    if (drivetrain_ != nullptr)
    {
        // Use Drivetrain's position tracking (average of both wheels)
        float left_mm  = drivetrain_->position(WheelSide::LEFT);
        float right_mm = drivetrain_->position(WheelSide::RIGHT);
        return (left_mm + right_mm) / 2.0f;
    }
    // Standalone mode: calculate from encoder ticks
    int32_t left_ticks  = left_encoder_->ticks();
    int32_t right_ticks = right_encoder_->ticks();
    float   avg_ticks   = static_cast<float>(left_ticks + right_ticks) / 2.0f;
    return avg_ticks * MM_PER_TICK;
}

float MotorLab::encoderVelocityMMps() const
{
    if (drivetrain_ != nullptr)
    {
        // Use Drivetrain's velocity tracking (returns mm/s, NOT distance!)
        float left_vel  = drivetrain_->velocity(WheelSide::LEFT);
        float right_vel = drivetrain_->velocity(WheelSide::RIGHT);
        return (left_vel + right_vel) / 2.0f;
    }
    // Standalone mode: use locally tracked velocity
    return (left_velocity_mmps_ + right_velocity_mmps_) / 2.0f;
}

void MotorLab::resetEncoders()
{
    if (drivetrain_ != nullptr)
    {
        drivetrain_->reset();
    }
    left_encoder_->reset();
    right_encoder_->reset();
    prev_left_ticks_     = 0;
    prev_right_ticks_    = 0;
    left_velocity_mmps_  = 0.0f;
    right_velocity_mmps_ = 0.0f;
}

void MotorLab::updateEncoders(float dt)
{
    // When using Drivetrain, velocities are updated externally via update()
    if (drivetrain_ != nullptr)
    {
        drivetrain_->update(dt);
        // Cache velocities for display in cmdEncoders
        left_velocity_mmps_  = drivetrain_->velocity(WheelSide::LEFT);
        right_velocity_mmps_ = drivetrain_->velocity(WheelSide::RIGHT);
        return;
    }

    // Standalone mode: calculate velocity from encoder delta
    if (dt < 0.001f)
    {
        return; // Avoid division by zero
    }

    int32_t left_ticks  = left_encoder_->ticks();
    int32_t right_ticks = right_encoder_->ticks();

    int32_t delta_left  = left_ticks - prev_left_ticks_;
    int32_t delta_right = right_ticks - prev_right_ticks_;

    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Convert tick deltas to mm/s (using MM_PER_TICK from Config.h)
    left_velocity_mmps_  = (delta_left * MM_PER_TICK) / dt;
    right_velocity_mmps_ = (delta_right * MM_PER_TICK) / dt;
}

// ============================================================================
// Test Routines
// ============================================================================

void MotorLab::runOpenLoopTrial(float max_voltage, float step_voltage, uint32_t settle_time_ms)
{
    printf("\n=== Open Loop Voltage Sweep ===\n");
    printf("Max voltage: %.2f V, Step: %.2f V, Settle time: %lu ms\n", max_voltage, step_voltage,
           static_cast<unsigned long>(settle_time_ms));
    printf("Battery: %.2f V\n\n", batteryVoltage());

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
            speed_sum += encoderVelocityMMps();
            sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
        }
        float avg_speed = speed_sum / static_cast<float>(num_samples);

        // Report data
        uint32_t now = to_ms_since_boot(get_absolute_time());
        reporter_.reportOpenLoop(now, voltage, avg_speed);
    }

    stopMotors();

    printf("\n=== Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("To find Km: slope of speed (mm/s) vs voltage line\n");
    printf("To find bias_ff: x-intercept of the line\n\n");
    printPrompt();
}

void MotorLab::runStepTrial(float step_voltage, uint32_t duration_ms)
{
    printf("\n=== Step Response Trial ===\n");
    printf("Step voltage: %.2f V, Duration: %lu ms\n", step_voltage,
           static_cast<unsigned long>(duration_ms));
    printf("Battery: %.2f V\n\n", batteryVoltage());

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
            reporter_.reportStep(now, step_voltage, encoderVelocityMMps(), encoderPositionMM());
        }

        sleep_ms(static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f));
    }

    stopMotors();

    printf("\n=== Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("To find Tm: time to reach 63.2%% of final speed\n\n");
    printPrompt();
}

void MotorLab::runMoveTrial(float distance, float top_speed, float acceleration, int mode)
{
    printf("\n=== Move Trial ===\n");
    printf("Distance: %.1f mm, Speed: %.1f mm/s, Accel: %.1f mm/s^2\n", distance, top_speed,
           acceleration);
    printf("Mode: %d (%s)\n", mode, mode == 0 ? "FF only" : (mode == 1 ? "PD only" : "FF+PD"));
    printf("Battery: %.2f V\n\n", batteryVoltage());

    reporter_.begin();
    reporter_.printControllerHeader();

    resetEncoders();

    // Start motion profile (in mm units)
    profile_.start(distance, top_speed, acceleration, 0.0f);

    // Initialize error accumulator for PD control
    float position_error = 0.0f;
    float prev_error     = 0.0f;

    // Control loop
    while (!profile_.finished())
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Update sensors
        updateEncoders(LOOP_INTERVAL_S);

        // Update profile
        profile_.update(LOOP_INTERVAL_S);

        // Get setpoints (in mm, mm/s, mm/s^2)
        float set_position = profile_.position();
        float set_speed    = profile_.speed();
        float set_accel    = profile_.acceleration();

        // Get actual values (in mm, mm/s)
        float actual_position = encoderPositionMM();
        float actual_speed    = encoderVelocityMMps();

        // Calculate control output
        float ff_volts      = 0.0f;
        float control_volts = 0.0f;

        // Feedforward (if enabled)
        if (mode == 0 || mode == 2)
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
        if (mode == 1 || mode == 2)
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

        // Total output
        float total_volts = ff_volts + control_volts;

        // Clamp to safe limits
        if (total_volts > MAX_VOLTAGE)
            total_volts = MAX_VOLTAGE;
        if (total_volts < -MAX_VOLTAGE)
            total_volts = -MAX_VOLTAGE;

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
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("Final position error: %.2f mm\n", profile_.position() - encoderPositionMM());
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
        int  c         = getchar_timeout_us(0); // Check USB
        bool from_uart = false;

        // Also check UART directly (for Bluetooth input)
        if (c == PICO_ERROR_TIMEOUT && uart_is_readable(uart0))
        {
            c         = uart_getc(uart0);
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
        if (from_uart)
        {
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

void MotorLab::executeCommand(const MotorLabArgs& args)
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
    printf("  KM [val]   - Motor velocity constant (mm/s/V)\n");
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
    printf("  ENC        - Show encoder values (mm, mm/s)\n");
    printf("  V [volts]  - Apply voltage to motors\n");
    printf("  X          - Stop motors\n");
    printf("\nTests:\n");
    printf("  OL [max] [step] [settle_ms] - Open-loop sweep (output: mm/s)\n");
    printf("  STEP [volts] [duration_ms]  - Step response (output: mm/s)\n");
    printf("  MOVE [mm] [mm/s] [mm/s^2] [mode] - Move trial\n");
    printf("       mode: 0=FF, 1=PD, 2=FF+PD\n");
    printf("\nExport:\n");
    printf("  EXPORT     - Print Config.h constants (mm/s format)\n");
    printf("\n");
}

void MotorLab::cmdId()
{
    printf("%s\n", MOTORLAB_VERSION);
    printf("Loop: %.0f Hz (%.1f ms)\n", LOOP_FREQUENCY_HZ, LOOP_INTERVAL_S * 1000.0f);
    printf("Max voltage: %.1f V\n", MAX_VOLTAGE);
    printf("Encoder: %.4f mm/tick\n", MM_PER_TICK);
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
        printf("speed_ff = %.7f V/(mm/s) (manual override)\n", settings_.speed_ff);
    }
    else if (args.argc == 1)
    {
        printf("speed_ff = %.7f V/(mm/s) [= 1/Km]\n", settings_.speed_ff);
    }
}

void MotorLab::cmdSetAccFF(const MotorLabArgs& args)
{
    // Accel FF is typically derived from Km and Tm, but allow override
    float val;
    if (parseFloat(args, 1, 0.0f, 0.01f, val))
    {
        settings_.acc_ff = val;
        printf("acc_ff = %.7f V/(mm/s^2) (manual override)\n", settings_.acc_ff);
    }
    else if (args.argc == 1)
    {
        printf("acc_ff = %.7f V/(mm/s^2) [= Tm/Km]\n", settings_.acc_ff);
    }
}

void MotorLab::cmdBattery()
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

void MotorLab::cmdEncoders()
{
    float left_mm  = left_encoder_->ticks() * MM_PER_TICK;
    float right_mm = right_encoder_->ticks() * MM_PER_TICK;

    printf("Position:\n");
    printf("  Left:  %ld ticks = %.1f mm\n", static_cast<long>(left_encoder_->ticks()), left_mm);
    printf("  Right: %ld ticks = %.1f mm\n", static_cast<long>(right_encoder_->ticks()), right_mm);

    printf("Velocity:\n");
    printf("  Avg: %.1f mm/s\n", encoderVelocityMMps());
    printf("  L: %.1f mm/s  R: %.1f mm/s\n", left_velocity_mmps_, right_velocity_mmps_);
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
    // Default values in mm units (half cell = 90mm, typical micromouse speeds)
    float dist  = 90.0f;  // mm (half cell)
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

void MotorLab::cmdVoltage(const MotorLabArgs& args)
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

void MotorLab::cmdVoltageLeft(const MotorLabArgs& args)
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

void MotorLab::cmdVoltageRight(const MotorLabArgs& args)
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

void MotorLab::cmdStop()
{
    stopMotors();
    printf("Motors stopped\n");
}

void MotorLab::cmdExport()
{
    // Convert MotorLab calibration values to Config.h format
    // MotorLab uses: mm/s, volts
    // Config.h uses: mm/s, duty cycle

    float battery_volts = batteryVoltage();

    // KV: duty per mm/s
    // duty = volts / battery, and volts = speed / km (where km is mm/s per volt)
    // So: duty_per_mmps = 1 / (km * battery_volts)
    float kv = 1.0f / (settings_.km * battery_volts);

    // KS: duty for static friction
    float ks = settings_.bias_ff / battery_volts;

    // Ka: acceleration feedforward (duty per mm/s^2)
    // acc_ff is in V/(mm/s^2), convert to duty/(mm/s^2)
    float ka = settings_.acc_ff / battery_volts;

    printf("\n");
    printf("// ============================================================\n");
    printf("// MotorLab Export - Copy to Config.h\n");
    printf("// Calibrated at battery voltage: %.2f V\n", battery_volts);
    printf("// ============================================================\n");
    printf("\n");
    printf("// Feedforward constants (duty-based, for mm/s units)\n");
    printf("#define FORWARD_KVL %.6ff  // Left velocity gain (duty per mm/s)\n", kv);
    printf("#define FORWARD_KVR %.6ff  // Right velocity gain (duty per mm/s)\n", kv);
    printf("#define FORWARD_KSL %.3ff     // Left static friction (duty)\n", ks);
    printf("#define FORWARD_KSR %.3ff     // Right static friction (duty)\n", ks);
    printf("\n");
    printf("// Acceleration feedforward (optional, set to 0 if not used)\n");
    printf("#define FORWARD_KAL %.7ff  // Left accel gain (duty per mm/s^2)\n", ka);
    printf("#define FORWARD_KAR %.7ff  // Right accel gain (duty per mm/s^2)\n", ka);
    printf("\n");
    printf("// Raw MotorLab values (for reference):\n");
    printf("//   Km = %.2f mm/s/V\n", settings_.km);
    printf("//   Tm = %.5f s\n", settings_.tm);
    printf("//   bias_ff = %.3f V\n", settings_.bias_ff);
    printf("// ============================================================\n");
    printf("\n");
}
