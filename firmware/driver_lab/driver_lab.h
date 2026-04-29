#ifndef DRIVER_LAB_DRIVER_LAB_H
#define DRIVER_LAB_DRIVER_LAB_H

#include "config/config.h"
#include "driver_lab/reporter.h"
#include "driver_lab/settings.h"

#include "pico/stdlib.h"

#include <cstdint>

class Encoder;
class Motor;
class Battery;
class Robot;
class ToF;
class LineSensor;

constexpr int DRIVERLAB_INPUT_BUFFER_SIZE = 64;
constexpr int DRIVERLAB_MAX_ARGC          = 8;
constexpr int DRIVERLAB_HISTORY_SIZE      = 10;

struct DriverLabArgs
{
    int   argc;
    char* argv[DRIVERLAB_MAX_ARGC];
};

/**
 * @brief UKMARS-style motor characterization and tuning interface
 *
 * Provides CLI commands for motor calibration:
 *   - Open-loop voltage sweeps (for Km, bias_ff calibration)
 *   - Step response tests (for Tm tuning)
 *   - Closed-loop move trials (for validating feedforward + controller)
 */
class DriverLab
{
  public:
    // Robot mode: direct motor/encoder + Robot access (for yaw/omega)
    DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
              Battery* battery, Robot* robot);

    // Robot mode with ToF sensors: direct motor/encoder + Robot + ToF access
    DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
              Battery* battery, Robot* robot, ToF* left_tof, ToF* front_tof, ToF* right_tof);

    // Robot mode with LineSensor: direct motor/encoder + Robot + LineSensor access
    DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
              Battery* battery, Robot* robot, LineSensor* line_sensor);

    void init();
    bool processSerial();

    // Inject a pre-read command line (no trailing newline) and run it
    // through DriverLab's existing tokenizer + dispatcher. Used by `Cli`
    // so the unified command loop can hand alpha tokens (OL, STEP,
    // EXPORT, MOVE, TURN, …) to DriverLab without DriverLab owning the
    // serial reader. Lines longer than `DRIVERLAB_INPUT_BUFFER_SIZE - 1`
    // are silently truncated.
    void executeLine(const char* line);

    DriverLabSettings& settings() { return settings_; }

    // Motor control
    void stopMotors();
    void setMotorVoltage(float volts);
    void setLeftMotorVoltage(float volts);
    void setRightMotorVoltage(float volts);

    float batteryVoltage() const;

    // Test routines
    void runOpenLoopTrial(float max_voltage = 6.0f, float step_voltage = 0.5f,
                          uint32_t settle_time_ms = 500);
    void runStepTrial(float step_voltage = 3.0f, uint32_t duration_ms = 1000);
    void runMoveTrial(float distance = 90.0f, float top_speed = 200.0f, float acceleration = 500.0f,
                      int mode = 2);
    void runTurnTrial(float degrees, float top_omega, float alpha);

    // TURN-OL: open-loop differential voltage sweep. Measures rotational kM
    // (deg/s per volt of differential drive). Robot is forced Idle; left and
    // right motors are commanded to ±diff_v so the robot rotates in place.
    void runTurnOpenLoopTrial(float max_diff_voltage = 3.0f, float step_voltage = 0.5f,
                              uint32_t settle_time_ms = 800);

    // TURN-STEP: differential-voltage step. Measures rotational time constant
    // ROT_TM from the 63.2% rise time of yaw rate (deg/s) to its steady state.
    void runTurnStepTrial(float diff_voltage = 1.5f, uint32_t duration_ms = 1000);

    // CLI commands
    void cmdHelp();
    void cmdId();
    void cmdSettings();
    void cmdInitSettings();
    void cmdSetKm(const DriverLabArgs& args);
    void cmdSetTm(const DriverLabArgs& args);
    void cmdSetZeta(const DriverLabArgs& args);
    void cmdSetTd(const DriverLabArgs& args);
    void cmdSetKp(const DriverLabArgs& args);
    void cmdSetKd(const DriverLabArgs& args);
    void cmdSetBiasFF(const DriverLabArgs& args);
    void cmdSetSpeedFF(const DriverLabArgs& args);
    void cmdSetAccFF(const DriverLabArgs& args);
    void cmdBattery();
    void cmdEncoders();
    void cmdYaw();
    void cmdYawVel();
    void cmdYawContinuous(const DriverLabArgs& args);
    void cmdYawReset();
    void cmdLeftTof();
    void cmdFrontTof();
    void cmdRightTof();
    void cmdTofContinuous(const DriverLabArgs& args);
    void cmdLeftEncoder();
    void cmdRightEncoder();
    void cmdEncoderReset();
    void cmdEncoderContinuous(const DriverLabArgs& args);
    void cmdOpenLoop(const DriverLabArgs& args);
    void cmdStep(const DriverLabArgs& args);
    void cmdMove(const DriverLabArgs& args);
    void cmdTurn(const DriverLabArgs& args);
    void cmdTurnOpenLoop(const DriverLabArgs& args);
    void cmdTurnStep(const DriverLabArgs& args);
    void cmdSetTurnKp(const DriverLabArgs& args);
    void cmdSetTurnKd(const DriverLabArgs& args);
    void cmdVoltage(const DriverLabArgs& args);
    void cmdVoltageLeft(const DriverLabArgs& args);
    void cmdVoltageRight(const DriverLabArgs& args);
    void cmdStop();
    void cmdExport();
    void cmdGpioDiag(const DriverLabArgs& args);
    void cmdDirTest(const DriverLabArgs& args);

    // Line sensor commands
    void cmdLinePosition();
    void cmdLineIntersection();
    void cmdLineContinuous(const DriverLabArgs& args);

  private:
    Motor*      left_motor_;
    Motor*      right_motor_;
    Encoder*    left_encoder_;
    Encoder*    right_encoder_;
    Battery*    battery_;
    Robot*      robot_;
    ToF*        left_tof_;
    ToF*        front_tof_;
    ToF*        right_tof_;
    LineSensor* line_sensor_;

    DriverLabSettings settings_;
    DriverLabReporter reporter_;

    char input_buffer_[DRIVERLAB_INPUT_BUFFER_SIZE];
    int  input_index_;
    bool echo_enabled_;

    // Command history (circular buffer)
    char history_[DRIVERLAB_HISTORY_SIZE][DRIVERLAB_INPUT_BUFFER_SIZE];
    int  history_count_;
    int  history_write_idx_;
    int  history_nav_idx_;
    char temp_buffer_[DRIVERLAB_INPUT_BUFFER_SIZE];

    int           readSerialLine();
    DriverLabArgs tokenize();
    void          executeCommand(const DriverLabArgs& args);
    void          clearInput();
    void          printPrompt();
    bool          parseFloat(const DriverLabArgs& args, int index, float min_val, float max_val,
                             float& result);

    // History commands and helpers
    void cmdRepeat();
    void cmdHistory();
    void executeHistorySelection(int selection);
    void saveToHistory();
};

#endif
