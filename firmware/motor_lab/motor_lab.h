#ifndef MOTOR_LAB_MOTOR_LAB_H
#define MOTOR_LAB_MOTOR_LAB_H

#include "config/config.h"
#include "motor_lab/profile.h"
#include "motor_lab/reporter.h"
#include "motor_lab/settings.h"

#include <cstdint>

class Drivetrain;
class Encoder;
class Motor;
class Battery;

constexpr int MOTORLAB_INPUT_BUFFER_SIZE = 64;
constexpr int MOTORLAB_MAX_ARGC = 8;

struct MotorLabArgs
{
    int argc;
    char* argv[MOTORLAB_MAX_ARGC];
};

/**
 * @brief UKMARS-style motor characterization and tuning interface
 *
 * Provides CLI commands for motor calibration:
 *   - Open-loop voltage sweeps (for Km, bias_ff calibration)
 *   - Step response tests (for Tm tuning)
 *   - Closed-loop move trials (for validating feedforward + controller)
 */
class MotorLab
{
  public:
    // Standalone mode: direct motor/encoder access
    MotorLab(Motor* left_motor, Motor* right_motor,
             Encoder* left_encoder, Encoder* right_encoder,
             Battery* battery);

    // Integrated mode: use Drivetrain
    MotorLab(Drivetrain* drivetrain,
             Encoder* left_encoder, Encoder* right_encoder,
             Battery* battery);

    void init();
    bool processSerial();

    MotorLabSettings& settings() { return settings_; }

    // Motor control
    void stopMotors();
    void setMotorVoltage(float volts);
    void setLeftMotorVoltage(float volts);
    void setRightMotorVoltage(float volts);

    float batteryVoltage() const;
    float encoderPositionMM() const;
    float encoderVelocityMMps() const;
    void resetEncoders();
    void updateEncoders(float dt);

    // Test routines
    void runOpenLoopTrial(float max_voltage = 6.0f, float step_voltage = 0.5f,
                          uint32_t settle_time_ms = 500);
    void runStepTrial(float step_voltage = 3.0f, uint32_t duration_ms = 1000);
    void runMoveTrial(float distance = 90.0f, float top_speed = 200.0f,
                      float acceleration = 500.0f, int mode = 2);

    // CLI commands
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
    void cmdExport();

  private:
    Motor* left_motor_;
    Motor* right_motor_;
    Encoder* left_encoder_;
    Encoder* right_encoder_;
    Battery* battery_;
    Drivetrain* drivetrain_;

    MotorLabSettings settings_;
    MotorLabProfile profile_;
    MotorLabReporter reporter_;

    char input_buffer_[MOTORLAB_INPUT_BUFFER_SIZE];
    int input_index_;
    bool echo_enabled_;

    int32_t prev_left_ticks_;
    int32_t prev_right_ticks_;
    float left_velocity_mmps_;
    float right_velocity_mmps_;

    int readSerialLine();
    MotorLabArgs tokenize();
    void executeCommand(const MotorLabArgs& args);
    void clearInput();
    void printPrompt();
    bool parseFloat(const MotorLabArgs& args, int index,
                    float min_val, float max_val, float& result);
};

#endif
