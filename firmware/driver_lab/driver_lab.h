#ifndef DRIVER_LAB_DRIVER_LAB_H
#define DRIVER_LAB_DRIVER_LAB_H

#include "config/config.h"
#include "control/pid.h"
#include "driver_lab/profile.h"
#include "driver_lab/reporter.h"
#include "driver_lab/settings.h"

#include "pico/stdlib.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class Encoder;
class Motor;
class Battery;
class IMU;
class ToF;
class LineSensor;

constexpr int DRIVERLAB_INPUT_BUFFER_SIZE = 64;
constexpr int DRIVERLAB_MAX_ARGC          = 24;
constexpr int DRIVERLAB_HISTORY_SIZE      = 10;

struct DriverLabArgs
{
    int   argc;
    char* argv[DRIVERLAB_MAX_ARGC];
};

/**
 * @brief Standalone calibration / characterization tool — modeled after
 *        Peter Harrison's MotorLab.
 *
 * DriverLab does NOT use Robot. It owns its own profiles, PD controllers,
 * and trial state machines, and talks directly to the HAL drivers (Motor,
 * Encoder, IMU, ToF, Battery). This mirrors the mazerunner-core / motorlab
 * split: shared HAL, separate control logic.
 *
 * Trials are cooperative state machines driven from a 500 Hz tick(). A
 * trial command (OL, STEP, MOVE, TURN, TURN-OL, TURN-STEP) prints its
 * header, arms the appropriate `TrialState`, then returns. Subsequent
 * tick() calls advance the trial without blocking the main loop.
 */
class DriverLab
{
  public:
    DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
              IMU* imu, Battery* battery, ToF* left_tof = nullptr, ToF* front_tof = nullptr,
              ToF* right_tof = nullptr, LineSensor* line_sensor = nullptr);

    void init();

    /**
     * @brief Advance one 500 Hz tick. Always call from the main loop.
     *
     * Refreshes encoder velocity cache and dispatches the active trial
     * (if any). When `trial_ == TrialState::Idle`, only the velocity
     * cache is updated — motors stay at whatever the last command set.
     */
    void tick();

    bool processSerial();

    // Inject a pre-read command line (no trailing newline) and run it
    // through DriverLab's tokenizer + dispatcher. Used by `Cli` so the
    // unified command loop can hand alpha tokens (OL, STEP, EXPORT,
    // MOVE, TURN, ...) to DriverLab without DriverLab owning the serial
    // reader. Lines longer than `DRIVERLAB_INPUT_BUFFER_SIZE - 1` are
    // silently truncated.
    void executeLine(const char* line);

    DriverLabSettings& settings() { return settings_; }

    // Motor control
    void stopMotors();
    void setMotorVoltage(float volts);
    void setLeftMotorVoltage(float volts);
    void setRightMotorVoltage(float volts);

    float batteryVoltage() const;

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
    void cmdPath(const DriverLabArgs& args);
    void cmdCenter(const DriverLabArgs& args);
    void cmdSetRotZeta(const DriverLabArgs& args);
    void cmdSetRotTd(const DriverLabArgs& args);
    void cmdSetTurnKp(const DriverLabArgs& args);
    void cmdSetTurnKd(const DriverLabArgs& args);
    void cmdVoltage(const DriverLabArgs& args);
    void cmdVoltageLeft(const DriverLabArgs& args);
    void cmdVoltageRight(const DriverLabArgs& args);
    void cmdStop();
    void cmdBtLog(const DriverLabArgs& args);
    void cmdExport();
    void cmdGpioDiag(const DriverLabArgs& args);
    void cmdDirTest(const DriverLabArgs& args);

    // Line sensor commands
    void cmdLinePosition();
    void cmdLineIntersection();
    void cmdLineContinuous(const DriverLabArgs& args);

  private:
    // ------------------------------------------------------------------
    // Trial state machine. Each enum value has a matching {start,tick,
    // finish} triple below. tick() dispatches on this enum.
    // ------------------------------------------------------------------
    enum class TrialState
    {
        Idle,
        Countdown,    // Cooperative 3-second pre-trial pause; arms next_state at GO!
        OpenLoop,     // OL:        voltage sweep
        Step,         // STEP:      forward step response
        Move,         // MOVE:      closed-loop forward profile
        Turn,         // TURN:      closed-loop rotation profile
        TurnOpenLoop, // TURN-OL:   differential-voltage sweep
        TurnStep,     // TURN-STEP: differential-voltage step response
        Path,         // PATH:      no-ToF known path with IMU heading hold
    };

    // Cooperative countdown: paced by the 500 Hz tick() loop, so the main
    // loop's sleep_until cadence never falls behind. Replaces the old
    // blocking sleep_ms(1000)*3 announcer that broke trial loop_count
    // semantics for the next several seconds. See CLAUDE.md "Loop dt".
    struct CountdownTrial
    {
        int        seconds_remaining; // 3 -> 2 -> 1 -> 0 (arm)
        int        tick_in_second;    // 0..LOOP_FREQUENCY_HZ-1
        TrialState next_state;        // which trial to arm at GO!
    };

    // Per-trial state structs. All accumulators live here; nothing leaks
    // out as a class member of DriverLab itself.
    struct OpenLoopTrial
    {
        // Trial parameters
        float    max_voltage;
        float    step_voltage;
        uint32_t settle_loops;

        // Per-step runtime state
        float current_voltage;
        int   step_loop_count; // ticks since this voltage step armed
        int   step_index;      // 0-based step number (for headers)

        // Velocity-window state for trailing-window speed estimation.
        // Even with deterministic dt, a 5-tick window smooths quantization
        // from int encoder ticks at low speeds.
        static constexpr int VEL_WIN = 5;
        float                left_pos_buf[VEL_WIN];
        float                right_pos_buf[VEL_WIN];
        int                  v_idx;
        int                  v_count;

        // Heading-hold reference: yaw at trial arm. Steering trim drives
        // (yaw_now - yaw_initial_deg) -> 0 to keep the robot tracking
        // straight during the multi-step voltage sweep. See
        // OL_STEERING_* in settings.h.
        float yaw_initial_deg;

        // Per-step rolling buffers (current step only)
        std::vector<float> step_left_speeds;
        std::vector<float> step_right_speeds;
        std::vector<float> step_left_volts;
        std::vector<float> step_right_volts;

        // Aggregate (across all steps)
        std::vector<float> left_voltages;
        std::vector<float> left_speeds;
        std::vector<float> right_voltages;
        std::vector<float> right_speeds;
        std::vector<float> combined_voltages;
        std::vector<float> combined_speeds;
        std::vector<float> tm_samples;
    };

    struct StepTrial
    {
        float voltage;
        int   duration_loops;
        int   loop_count;

        // 5-tick trailing-window velocity (mirrors OpenLoopTrial::VEL_WIN).
        // Smooths integer encoder quantization at the 2 ms loop rate; without
        // this, dense Tm-fit samples would be dominated by tick-count noise.
        static constexpr int VEL_WIN = 5;
        float                left_pos_buf[VEL_WIN];
        float                right_pos_buf[VEL_WIN];
        int                  v_idx;
        int                  v_count;

        std::vector<float> times_s;
        std::vector<float> speeds;
    };

    struct MoveTrial
    {
        float distance_mm;
        float top_speed;
        float acceleration;
        int   mode; // 0=FF only, 1=PD only, 2=FF+PD

        // Diagnostics
        float max_speed_error;
        float sum_speed_error;
        int   diag_count;
        float max_pos_error;
        float max_volts;
        int   loop_count;
    };

    struct TurnTrial
    {
        float degrees;
        float top_omega;
        float alpha;

        // Diagnostics
        float peak_overshoot_deg;
        float max_yaw_error;
        float max_volts;
        float old_left_speed_mmps;
        float old_right_speed_mmps;
        bool  past_target;
        int   loop_count;
    };

    struct TurnOpenLoopTrial
    {
        float    max_voltage;
        float    step_voltage;
        uint32_t settle_loops;

        float current_voltage;
        int   step_loop_count;
        int   step_index;
        float yaw_at_step_start;

        std::vector<float> step_omegas;
        std::vector<float> step_times_s;

        std::vector<float> diff_voltages;
        std::vector<float> omega_steady;
        std::vector<float> rot_tm_samples;

        // Inter-step pause
        bool pausing;
        int  pause_loop_count;
    };

    struct TurnStepTrial
    {
        float    diff_voltage;
        int      duration_loops;
        int      loop_count;
        uint32_t last_packet_seq; // gates Tm-fit sample push to true IMU cadence

        std::vector<float> times_s;
        std::vector<float> omegas;
    };

    enum class PathSegmentType
    {
        Forward,
        Turn,
        SmoothTurn
    };

    struct PathSegment
    {
        PathSegmentType type;
        float           value; // Forward: mm. Turns: deg.
    };

    struct PathTrial
    {
        std::vector<PathSegment> segments;
        size_t                   segment_index;
        float                    speed_mmps;
        float                    accel_mmps2;
        float                    omega_degps;
        float                    alpha_degps2;
        bool                     smooth_turns;
        float                    expected_yaw_deg;
        float                    total_forward_mm;
        float                    max_volts;
        float                    old_left_speed_mmps;
        float                    old_right_speed_mmps;
        int                      segment_count;
        int                      loop_count;
    };

    // ------------------------------------------------------------------
    // Hardware
    // ------------------------------------------------------------------
    Motor*      left_motor_;
    Motor*      right_motor_;
    Encoder*    left_encoder_;
    Encoder*    right_encoder_;
    IMU*        imu_;
    Battery*    battery_;
    ToF*        left_tof_;
    ToF*        front_tof_;
    ToF*        right_tof_;
    LineSensor* line_sensor_;

    DriverLabSettings settings_;
    DriverLabReporter reporter_;

    // Own controllers + profiles, separate from any Robot instance.
    PID              forward_pid_;
    PID              rotation_pid_;
    DriverLabProfile forward_profile_;
    DriverLabProfile rotation_profile_;

    // Per-tick encoder velocity cache (computed in tick()).
    int   prev_left_ticks_;
    int   prev_right_ticks_;
    float left_velocity_mmps_;
    float right_velocity_mmps_;
    float left_position_mm_;
    float right_position_mm_;

    // 8-tap moving averager state, parallel to Drivetrain's. DriverLab reads
    // encoders directly (HAL-sharing rule) so it owns its own MA state and
    // applies the same algorithm — gains tuned in DriverLab transfer cleanly
    // to maze-running because both paths see the same smoothed signal.
    int8_t dl_left_history_[ENCODER_AVERAGER_LENGTH]  = {};
    int8_t dl_right_history_[ENCODER_AVERAGER_LENGTH] = {};
    int    dl_left_history_total_                     = 0;
    int    dl_right_history_total_                    = 0;
    int    dl_averager_index_                         = 0;

    // Active trial dispatch
    TrialState        trial_;
    CountdownTrial    countdown_;
    OpenLoopTrial     ol_;
    StepTrial         step_;
    MoveTrial         move_;
    TurnTrial         turn_;
    TurnOpenLoopTrial tol_;
    TurnStepTrial     tstep_;
    PathTrial         path_;

    // ------------------------------------------------------------------
    // CLI buffers
    // ------------------------------------------------------------------
    char input_buffer_[DRIVERLAB_INPUT_BUFFER_SIZE];
    int  input_index_;
    bool echo_enabled_;

    char history_[DRIVERLAB_HISTORY_SIZE][DRIVERLAB_INPUT_BUFFER_SIZE];
    int  history_count_;
    int  history_write_idx_;
    int  history_nav_idx_;
    char temp_buffer_[DRIVERLAB_INPUT_BUFFER_SIZE];

    // ------------------------------------------------------------------
    // Per-tick helpers
    // ------------------------------------------------------------------
    void  sampleEncoders(); // refresh L/R velocity + position cache
    void  resetEncoderMA(); // clear 8-tap MA ring buffers (call on trial-arm)
    void  setVoltages(float lv, float rv);
    float feedforwardVolts(float speed_mmps, float accel_mmps2, bool left) const;

    // ------------------------------------------------------------------
    // Trial entry / per-tick / exit triples
    // ------------------------------------------------------------------
    void startOpenLoopTrial(float max_v, float step_v, uint32_t settle_ms);
    void tickOpenLoop();
    void finishOpenLoop();

    void startStepTrial(float volts, uint32_t duration_ms);
    void tickStep();
    void finishStep();

    void startMoveTrial(float distance, float speed, float accel, int mode);
    void tickMove();
    void finishMove();

    void startTurnTrial(float degrees, float omega, float alpha);
    void tickTurn();
    void finishTurn();

    void startTurnOpenLoopTrial(float max_v, float step_v, uint32_t settle_ms);
    void tickTurnOpenLoop();
    void finishTurnOpenLoop();

    void startTurnStepTrial(float diff_v, uint32_t duration_ms);
    void tickTurnStep();
    void finishTurnStep();

    bool parsePathSequence(const DriverLabArgs& args, int first_param_index);
    bool parsePathChunk(const char* chunk);
    void appendPathForwardCells(int cells);
    void appendPathTurn(float degrees);
    void startPathTrial(float speed_mmps, float accel_mmps2, float omega_degps, float alpha_degps2,
                        bool smooth_turns);
    void armPath();
    void tickPath();
    void finishPath();
    void startNextPathSegment();
    void tickPathForward(const PathSegment& segment);
    void tickPathTurn(const PathSegment& segment);
    void tickPathSmoothTurn(const PathSegment& segment);
    void printBluetoothDiagnostics();

    // Cooperative countdown: announces the trial, then trial_ = Countdown
    // and tickCountdown() decrements once per LOOP_FREQUENCY_HZ ticks. At
    // GO!, tickCountdown() calls the matching arm*() to actually start
    // motors / reset state. Replaces the old blocking countdownAndAnnounce.
    void beginCountdown(const char* trial_name, TrialState next);
    void tickCountdown();

    // arm*() = the "GO!" half of each trial: encoder/IMU reset, initial
    // motor commands, controller resets, reporter setup, trial_ transition.
    // Parameters live in the per-trial struct (set by start*Trial() before
    // the countdown begins).
    void armOpenLoop();
    void armStep();
    void armMove();
    void armTurn();
    void armTurnOpenLoop();
    void armTurnStep();

    // ------------------------------------------------------------------
    // CLI plumbing
    // ------------------------------------------------------------------
    int           readSerialLine();
    DriverLabArgs tokenize();
    void          executeCommand(const DriverLabArgs& args);
    void          clearInput();
    void          printPrompt();
    bool          parseFloat(const DriverLabArgs& args, int index, float min_val, float max_val,
                             float& result);

    void cmdRepeat();
    void cmdHistory();
    void executeHistorySelection(int selection);
    void saveToHistory();
};

#endif
