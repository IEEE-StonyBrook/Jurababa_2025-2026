#ifndef MOTORLAB_H
#define MOTORLAB_H

#include "Platform/Pico/Config.h"
#include "Platform/Pico/MotorLab/MotorLabProfile.h"
#include "Platform/Pico/MotorLab/MotorLabReporter.h"
#include "Platform/Pico/MotorLab/MotorLabSettings.h"

#include <cstdint>

// Forward declarations
class Drivetrain;
class Encoder;
class Motor;
class BatteryMonitor;

/**
 * @file MotorLab.h
 * @brief UKMARS-style motor characterization and tuning interface
 *
 * Provides CLI commands and test routines for motor calibration:
 *   - Open-loop voltage sweeps (for Km, bias_ff calibration)
 *   - Step response tests (for Tm, zeta, Td tuning)
 *   - Closed-loop move trials (for validating feedforward + controller)
 *
 * Based on UKMARS motorlab by Peter Harrison.
 */

/**
 * @brief CLI command buffer size
 */
constexpr int MOTORLAB_INPUT_BUFFER_SIZE = 64;

/**
 * @brief Maximum number of CLI arguments
 */
constexpr int MOTORLAB_MAX_ARGC = 8;

/**
 * @brief CLI argument structure
 */
struct MotorLabArgs
{
    int   argc;
    char* argv[MOTORLAB_MAX_ARGC];
};

/**
 * @brief Motor lab interface for characterization and tuning
 *
 * Provides serial CLI for running motor tests and adjusting parameters.
 * Designed to work with the existing Drivetrain and Motor classes.
 */
class MotorLab
{
  public:
    /**
     * @brief Construct motor lab with hardware references
     *
     * @param left_motor Pointer to left motor
     * @param right_motor Pointer to right motor
     * @param left_encoder Pointer to left encoder
     * @param right_encoder Pointer to right encoder
     * @param battery_monitor Pointer to battery monitor
     */
    MotorLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
             BatteryMonitor* battery_monitor);

    /**
     * @brief Initialize motor lab (call once at startup)
     */
    void init();

    /**
     * @brief Process serial input (call in main loop)
     *
     * Reads characters from serial, parses commands when complete line
     * is received, and executes the appropriate action.
     *
     * @return true if a command was processed, false otherwise
     */
    bool processSerial();

    /**
     * @brief Get current settings
     * @return Reference to settings structure
     */
    MotorLabSettings& getSettings() { return settings_; }

    // ========================================================================
    // Motor Control Methods (used during trials)
    // ========================================================================

    /**
     * @brief Stop both motors immediately
     */
    void stopMotors();

    /**
     * @brief Apply voltage to both motors equally
     * @param volts Voltage to apply (battery-compensated)
     */
    void setMotorVoltage(float volts);

    /**
     * @brief Apply voltage to left motor only (right motor stopped)
     * @param volts Voltage to apply (battery-compensated)
     */
    void setLeftMotorVoltage(float volts);

    /**
     * @brief Apply voltage to right motor only (left motor stopped)
     * @param volts Voltage to apply (battery-compensated)
     */
    void setRightMotorVoltage(float volts);

    /**
     * @brief Get current battery voltage
     * @return Battery voltage in volts
     */
    float getBatteryVoltage() const;

    /**
     * @brief Get encoder position in degrees
     *
     * Uses average of both encoders for combined measurement.
     *
     * @return Position in degrees
     */
    float getEncoderPositionDeg() const;

    /**
     * @brief Get encoder velocity in degrees/second
     *
     * Uses average of both encoders for combined measurement.
     *
     * @return Velocity in deg/s
     */
    float getEncoderVelocityDegps() const;

    /**
     * @brief Reset encoder counts to zero
     */
    void resetEncoders();

    /**
     * @brief Update encoder velocity estimates
     *
     * Call this at a fixed rate (e.g., 100Hz) to compute velocities.
     *
     * @param dt Time step in seconds
     */
    void updateEncoders(float dt);

    // ========================================================================
    // Test Routines
    // ========================================================================

    /**
     * @brief Run open-loop voltage sweep test
     *
     * Applies increasing voltages and measures steady-state speed.
     * Use this to determine Km (motor velocity constant) and bias_ff.
     *
     * Results: Plot voltage vs speed. Slope = 1/Km, y-intercept = bias_ff
     *
     * @param max_voltage Maximum voltage to test (default 6V)
     * @param step_voltage Voltage increment (default 0.5V)
     * @param settle_time_ms Time to wait for steady state (default 500ms)
     */
    void runOpenLoopTrial(float max_voltage = 6.0f, float step_voltage = 0.5f,
                          uint32_t settle_time_ms = 500);

    /**
     * @brief Run step response test
     *
     * Applies a step voltage and records transient response.
     * Use this to determine Tm (motor time constant).
     *
     * Results: Find time to reach 63.2% of final speed. That time = Tm.
     *
     * @param step_voltage Step input voltage (default 3V)
     * @param duration_ms Test duration (default 1000ms)
     */
    void runStepTrial(float step_voltage = 3.0f, uint32_t duration_ms = 1000);

    /**
     * @brief Run closed-loop move test
     *
     * Executes a profiled move with feedforward and/or PD control.
     * Use this to validate tuning and compare control modes.
     *
     * @param distance Target distance in degrees
     * @param top_speed Maximum speed in deg/s
     * @param acceleration Acceleration in deg/s²
     * @param mode Control mode: 0=FF only, 1=PD only, 2=FF+PD
     */
    void runMoveTrial(float distance = 360.0f, float top_speed = 500.0f,
                      float acceleration = 1000.0f, int mode = 2);

    // ========================================================================
    // CLI Commands (called by processSerial)
    // ========================================================================

    void cmdHelp();
    void cmdId();
    void cmdSettings();
    void cmdInitSettings();
    void cmdSetKm(const MotorLabArgs& args);
    void cmdSetTm(const MotorLabArgs& args);
    void cmdSetZeta(const MotorLabArgs& args);
    void cmdSetTd(const MotorLabArgs& args);
    void cmdSetKp(const MotorLabArgs& args);
    void cmdSetKd(const MotorLabArgs& args);
    void cmdSetBiasFF(const MotorLabArgs& args);
    void cmdSetSpeedFF(const MotorLabArgs& args);
    void cmdSetAccFF(const MotorLabArgs& args);
    void cmdBattery();
    void cmdEncoders();
    void cmdOpenLoop(const MotorLabArgs& args);
    void cmdStep(const MotorLabArgs& args);
    void cmdMove(const MotorLabArgs& args);
    void cmdVoltage(const MotorLabArgs& args);
    void cmdVoltageLeft(const MotorLabArgs& args);
    void cmdVoltageRight(const MotorLabArgs& args);
    void cmdStop();

    // TODO: Add WRITE/READ commands for Flash persistence (UKMARS uses EEPROM.put/get)

  private:
    // Hardware references
    Motor*          left_motor_;
    Motor*          right_motor_;
    Encoder*        left_encoder_;
    Encoder*        right_encoder_;
    BatteryMonitor* battery_monitor_;

    // Settings and utilities
    MotorLabSettings settings_;
    MotorLabProfile  profile_;
    MotorLabReporter reporter_;

    // CLI input buffer
    char input_buffer_[MOTORLAB_INPUT_BUFFER_SIZE];
    int  input_index_;
    bool echo_enabled_;

    // Encoder velocity tracking
    int32_t prev_left_ticks_;
    int32_t prev_right_ticks_;
    float   left_velocity_degps_;
    float   right_velocity_degps_;

    // CLI parsing helpers
    int          readSerialLine();
    MotorLabArgs tokenize();
    void         executeCommand(const MotorLabArgs& args);
    void         clearInput();
    void         printPrompt();

    /**
     * @brief Helper to parse float argument with bounds checking
     * @param args Command arguments
     * @param index Argument index to parse
     * @param min_val Minimum allowed value
     * @param max_val Maximum allowed value
     * @param result Output parameter for parsed value
     * @return true if valid, false otherwise
     */
    bool parseFloat(const MotorLabArgs& args, int index, float min_val, float max_val,
                    float& result);
};

#endif // MOTORLAB_H
