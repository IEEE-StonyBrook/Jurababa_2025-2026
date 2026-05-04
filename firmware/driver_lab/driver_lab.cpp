#include "driver_lab/driver_lab.h"

#include "common/bluetooth_stdio.h"
#include "common/utils.h"
#include "config/config.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/line_sensor.h"
#include "drivers/motor.h"
#include "drivers/tof.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ============================================================================
// Version and identification
// ============================================================================
static const char* DRIVERLAB_VERSION = "DRIVERLAB v3.0 (Jurababa, standalone)";

namespace
{
// Convert a duration in milliseconds to a number of 500 Hz loop ticks.
// Equivalent to ms * LOOP_FREQUENCY_HZ / 1000, but written in the rounding-
// safe form `(ms * Hz + 999) / 1000` so a 1 ms request never floors to 0.
inline uint32_t msToLoops(uint32_t ms)
{
    const uint32_t hz = static_cast<uint32_t>(LOOP_FREQUENCY_HZ);
    return (ms * hz + 999u) / 1000u;
}

inline float loopsToSeconds(int loops)
{
    return loops * LOOP_INTERVAL_S;
}

inline float normalizeYawDelta(float delta)
{
    if (delta > 180.0f)
        return delta - 360.0f;
    if (delta < -180.0f)
        return delta + 360.0f;
    return delta;
}
} // namespace

// ============================================================================
// Constructor and Initialization
// ============================================================================

DriverLab::DriverLab(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                     Encoder* right_encoder, IMU* imu, Battery* battery, ToF* left_tof,
                     ToF* front_tof, ToF* right_tof, LineSensor* line_sensor)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), imu_(imu), battery_(battery), left_tof_(left_tof),
      front_tof_(front_tof), right_tof_(right_tof), line_sensor_(line_sensor), reporter_(10),
      // Forward PID runs at the encoder-paced 500 Hz rate. Rotation PID runs
      // at the IMU's 100 Hz packet rate; gated by imu_->has_new_yaw_sample()
      // in tickTurn(), with a zero-order hold between packets.
      forward_pid_(0.0f, 0.0f, LOOP_FREQUENCY_HZ), rotation_pid_(0.0f, 0.0f, ROTATION_LOOP_HZ),
      prev_left_ticks_(0), prev_right_ticks_(0), left_velocity_mmps_(0.0f),
      right_velocity_mmps_(0.0f), left_position_mm_(0.0f), right_position_mm_(0.0f),
      trial_(TrialState::Idle), input_index_(0), echo_enabled_(false), history_count_(0),
      history_write_idx_(0), history_nav_idx_(-1)
{
    countdown_.seconds_remaining = 0;
    countdown_.tick_in_second    = 0;
    countdown_.next_state        = TrialState::Idle;
    clearInput();
    temp_buffer_[0] = '\0';
}

void DriverLab::init()
{
    settings_.initDefaults();

    if (left_encoder_)
        left_encoder_->reset();
    if (right_encoder_)
        right_encoder_->reset();
    prev_left_ticks_   = 0;
    prev_right_ticks_  = 0;
    left_position_mm_  = 0.0f;
    right_position_mm_ = 0.0f;
    resetEncoderMA();

    forward_pid_.setOutputLimit(MAX_VOLTAGE);
    rotation_pid_.setOutputLimit(MAX_VOLTAGE);

    printf("\n%s\n", DRIVERLAB_VERSION);
    printf("Loop frequency: %.0f Hz (forward PD), %.0f Hz (rotation PD, IMU-gated)\n",
           LOOP_FREQUENCY_HZ, ROTATION_LOOP_HZ);
    printf("Using mm/s units (MM_PER_TICK = %.4f)\n", MM_PER_TICK);
    printf("Type '?' for help\n\n");
    printPrompt();
}

// ============================================================================
// Per-tick helpers
// ============================================================================

void DriverLab::sampleEncoders()
{
    if (!left_encoder_ || !right_encoder_)
        return;

    const int left_ticks  = left_encoder_->ticks();
    const int right_ticks = right_encoder_->ticks();

    const int d_left  = left_ticks - prev_left_ticks_;
    const int d_right = right_ticks - prev_right_ticks_;

    // 8-tap moving averager — same shape as Drivetrain::update() and
    // ukmars/motorlab/src/encoders.h::update(). Smoothing here means
    // forward_pid_'s measured_change input matches what Robot's PD sees
    // in maze mode, so DriverLab-tuned gains transfer.
    dl_left_history_total_ -= dl_left_history_[dl_averager_index_];
    dl_right_history_total_ -= dl_right_history_[dl_averager_index_];
    dl_left_history_total_ += d_left;
    dl_right_history_total_ += d_right;
    dl_left_history_[dl_averager_index_]  = static_cast<int8_t>(d_left);
    dl_right_history_[dl_averager_index_] = static_cast<int8_t>(d_right);
    dl_averager_index_                    = (dl_averager_index_ + 1) % ENCODER_AVERAGER_LENGTH;

    constexpr float inv_avg    = 1.0f / static_cast<float>(ENCODER_AVERAGER_LENGTH);
    const float left_change_mm = static_cast<float>(dl_left_history_total_) * inv_avg * MM_PER_TICK;
    const float right_change_mm =
        static_cast<float>(dl_right_history_total_) * inv_avg * MM_PER_TICK;

    // Deterministic dt: the main loop paces with sleep_until(next_tick), so
    // multiplying by the compile-time LOOP_FREQUENCY_HZ is exact. (See
    // CLAUDE.md "Loop dt: use LOOP_FREQUENCY_HZ, do not measure".)
    left_velocity_mmps_  = left_change_mm * LOOP_FREQUENCY_HZ;
    right_velocity_mmps_ = right_change_mm * LOOP_FREQUENCY_HZ;

    // Position accumulates the smoothed delta (mirrors MotorLab's
    // m_robot_distance += m_fwd_change). Integral of MA equals integral of
    // raw deltas modulo an 8-tick startup transient — Tm-fit ring buffers
    // sampling position remain unbiased.
    left_position_mm_ += left_change_mm;
    right_position_mm_ += right_change_mm;

    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;
}

void DriverLab::resetEncoderMA()
{
    for (int i = 0; i < ENCODER_AVERAGER_LENGTH; i++)
    {
        dl_left_history_[i]  = 0;
        dl_right_history_[i] = 0;
    }
    dl_left_history_total_  = 0;
    dl_right_history_total_ = 0;
    dl_averager_index_      = 0;
}

void DriverLab::setVoltages(float lv, float rv)
{
    const float batt = batteryVoltage();
    if (lv > MAX_VOLTAGE)
        lv = MAX_VOLTAGE;
    else if (lv < -MAX_VOLTAGE)
        lv = -MAX_VOLTAGE;
    if (rv > MAX_VOLTAGE)
        rv = MAX_VOLTAGE;
    else if (rv < -MAX_VOLTAGE)
        rv = -MAX_VOLTAGE;
    left_motor_->set_motor_volts(lv, batt);
    right_motor_->set_motor_volts(rv, batt);
}

float DriverLab::feedforwardVolts(float speed_mmps, float accel_mmps2, bool left) const
{
    // V = kV * speed + sign(speed) * kS + kA * accel  — mazerunner-core shape.
    const float kV   = left ? (1.0f / settings_.kM_L) : (1.0f / settings_.kM_R);
    const float kS   = left ? settings_.kS_L : settings_.kS_R;
    const float kA   = left ? settings_.kA_L : settings_.kA_R;
    const float bias = (speed_mmps > 0.1f) ? kS : ((speed_mmps < -0.1f) ? -kS : 0.0f);
    return kV * speed_mmps + bias + kA * accel_mmps2;
}

// ============================================================================
// tick() — called every 500 Hz from main.cpp
// ============================================================================

void DriverLab::tick()
{
    sampleEncoders();

    switch (trial_)
    {
        case TrialState::Idle:
            // Motors hold whatever the last command set; no controller fires.
            // Show commands and `V` voltage commands work outside any trial.
            break;
        case TrialState::Countdown:
            tickCountdown();
            break;
        case TrialState::OpenLoop:
            tickOpenLoop();
            break;
        case TrialState::Step:
            tickStep();
            break;
        case TrialState::Move:
            tickMove();
            break;
        case TrialState::Turn:
            tickTurn();
            break;
        case TrialState::TurnOpenLoop:
            tickTurnOpenLoop();
            break;
        case TrialState::TurnStep:
            tickTurnStep();
            break;
    }
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
    setVoltages(volts, volts);
}

void DriverLab::setLeftMotorVoltage(float volts)
{
    setVoltages(volts, 0.0f);
}

void DriverLab::setRightMotorVoltage(float volts)
{
    setVoltages(0.0f, volts);
}

float DriverLab::batteryVoltage() const
{
    return battery_->voltage();
}

// Cooperative 3-second pre-trial pause. Replaces the old blocking
// countdownAndAnnounce(): a sleep_ms(1000)*3 inside the same thread as the
// 500 Hz control loop pushed `next_tick` 3 s into the past, after which
// sleep_until() in main was a no-op and trial loop_count budgets blew
// through in microseconds. See CLAUDE.md "Loop dt" and the
// when-i-begin-driverlab-cryptic-duckling plan.
void DriverLab::beginCountdown(const char* trial_name, TrialState next)
{
    printf("\n=== %s ===\n", trial_name);
    printf("Battery: %.2f V\n", batteryVoltage());
    BluetoothStdio::resetDiagnostics();
    countdown_.seconds_remaining = 3;
    countdown_.tick_in_second    = 0;
    countdown_.next_state        = next;
    printf("Starting in %d...\n", countdown_.seconds_remaining);
    trial_ = TrialState::Countdown;
}

void DriverLab::tickCountdown()
{
    countdown_.tick_in_second++;
    if (countdown_.tick_in_second < static_cast<int>(LOOP_FREQUENCY_HZ))
        return;

    countdown_.tick_in_second = 0;
    countdown_.seconds_remaining--;

    if (countdown_.seconds_remaining > 0)
    {
        printf("Starting in %d...\n", countdown_.seconds_remaining);
        return;
    }

    printf("GO!\n\n");

    switch (countdown_.next_state)
    {
        case TrialState::OpenLoop:
            armOpenLoop();
            break;
        case TrialState::Step:
            armStep();
            break;
        case TrialState::Move:
            armMove();
            break;
        case TrialState::Turn:
            armTurn();
            break;
        case TrialState::TurnOpenLoop:
            armTurnOpenLoop();
            break;
        case TrialState::TurnStep:
            armTurnStep();
            break;
        default:
            // No matching trial — fall back to Idle. Should never happen
            // unless beginCountdown() was invoked with a non-trial state.
            trial_ = TrialState::Idle;
            break;
    }
}

// ============================================================================
// OL — open-loop voltage sweep (stereo, per-motor regression)
// ============================================================================

void DriverLab::startOpenLoopTrial(float max_v, float step_v, uint32_t settle_ms)
{
    printf("Max voltage: %.2f V, Step: %.2f V, Settle: %lu ms\n", max_v, step_v,
           static_cast<unsigned long>(settle_ms));

    ol_.max_voltage     = max_v;
    ol_.step_voltage    = step_v;
    ol_.settle_loops    = msToLoops(settle_ms);
    ol_.current_voltage = step_v;
    ol_.step_loop_count = 0;
    ol_.step_index      = 0;
    ol_.v_idx           = 0;
    ol_.v_count         = 0;
    for (int i = 0; i < OpenLoopTrial::VEL_WIN; ++i)
    {
        ol_.left_pos_buf[i]  = 0.0f;
        ol_.right_pos_buf[i] = 0.0f;
    }
    ol_.step_left_speeds.clear();
    ol_.step_right_speeds.clear();
    ol_.step_left_volts.clear();
    ol_.step_right_volts.clear();
    ol_.left_voltages.clear();
    ol_.left_speeds.clear();
    ol_.right_voltages.clear();
    ol_.right_speeds.clear();
    ol_.combined_voltages.clear();
    ol_.combined_speeds.clear();
    ol_.tm_samples.clear();

    beginCountdown("Open Loop Voltage Sweep (Stereo)", TrialState::OpenLoop);
}

void DriverLab::armOpenLoop()
{
    if (left_encoder_)
        left_encoder_->reset();
    if (right_encoder_)
        right_encoder_->reset();
    prev_left_ticks_   = 0;
    prev_right_ticks_  = 0;
    left_position_mm_  = 0.0f;
    right_position_mm_ = 0.0f;
    resetEncoderMA();

    // Capture starting heading as the steering trim's reference. Relative
    // (not absolute) so the trial works regardless of initial yaw, and we
    // avoid imu_->reset() to keep absolute yaw intact for any other consumer.
    ol_.yaw_initial_deg = imu_->robot_angle();

    reporter_.begin();
    reporter_.printOpenLoopStereoHeader();

    setVoltages(ol_.current_voltage, ol_.current_voltage);
    trial_ = TrialState::OpenLoop;
}

void DriverLab::tickOpenLoop()
{
    constexpr int   SKIP_SAMPLES        = 10; // 20 ms transient skip at 500 Hz
    constexpr int   OUTPUT_EVERY        = 5;  // CSV row every 5th retained sample
    constexpr float MIN_SPEED_THRESHOLD = 10.0f;

    // Trailing-window velocity from the oldest sample in the ring buffer.
    // Even with deterministic dt, a 5-tick window smooths integer encoder
    // quantization at low speeds.
    float left_speed = 0.0f, right_speed = 0.0f;
    if (ol_.v_count > 0)
    {
        const int oldest = (ol_.v_count < OpenLoopTrial::VEL_WIN) ? 0 : ol_.v_idx;
        const int span =
            (ol_.v_count < OpenLoopTrial::VEL_WIN) ? ol_.v_count : OpenLoopTrial::VEL_WIN;
        const float dt_s = span * LOOP_INTERVAL_S;
        if (dt_s > 1e-6f)
        {
            left_speed  = (left_position_mm_ - ol_.left_pos_buf[oldest]) / dt_s;
            right_speed = (right_position_mm_ - ol_.right_pos_buf[oldest]) / dt_s;
        }
    }

    ol_.left_pos_buf[ol_.v_idx]  = left_position_mm_;
    ol_.right_pos_buf[ol_.v_idx] = right_position_mm_;
    ol_.v_idx                    = (ol_.v_idx + 1) % OpenLoopTrial::VEL_WIN;
    if (ol_.v_count < OpenLoopTrial::VEL_WIN)
        ol_.v_count++;

    // Heading-hold steering trim: keep the robot tracking straight across
    // the multi-step sweep so it doesn't drift into walls. Sign convention
    // matches TURN-OL (setVoltages(-V, +V) -> CCW positive omega), so a
    // positive heading drift gets a positive trim -> left_v > right_v ->
    // CW correction.
    //
    // Manually tuned gains (independent of settings_.rot_kM) — see
    // settings.h. The earlier formula path scaled by an uncalibrated rot_kM
    // and pushed ±0.5 V trim oscillation; direct constants are stable.
    const float h_err_deg = utils::wrapAngle180(imu_->robot_angle() - ol_.yaw_initial_deg);
    const float trim_v    = utils::clampAbs(OL_STEERING_KP_VPDEG * h_err_deg +
                                                OL_STEERING_KD_VPDPS * imu_->robot_omega(),
                                            OL_STEERING_TRIM_MAX_V);
    const float left_v    = ol_.current_voltage + trim_v;
    const float right_v   = ol_.current_voltage - trim_v;
    setVoltages(left_v, right_v);

    if (ol_.step_loop_count >= SKIP_SAMPLES)
    {
        ol_.step_left_speeds.push_back(left_speed);
        ol_.step_right_speeds.push_back(right_speed);
        // Log the ACTUAL per-side voltage (post-trim), not the commanded
        // base. The per-motor regression in finishOpenLoop() consumes the
        // steady-state mean of these buffers, so any persistent trim bias
        // (e.g. from drivetrain asymmetry) shifts both regression axes
        // together along the true plant line -- kM/kS extract correctly
        // regardless of trim magnitude.
        ol_.step_left_volts.push_back(left_v);
        ol_.step_right_volts.push_back(right_v);

        const int kept = ol_.step_loop_count - SKIP_SAMPLES;
        if (kept % OUTPUT_EVERY == 0)
        {
            const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            reporter_.reportOpenLoopStereo(now_ms, ol_.current_voltage, left_v, right_v, left_speed,
                                           right_speed, imu_->robot_angle());
        }
    }

    ol_.step_loop_count++;
    if (static_cast<uint32_t>(ol_.step_loop_count) < ol_.settle_loops)
        return;

    // ------------------------ end of step ------------------------
    // Average last 10 retained samples = steady-state.
    const size_t avg_count = std::min(ol_.step_left_speeds.size(), size_t(10));
    float        left_ss = 0.0f, right_ss = 0.0f;
    if (avg_count > 0)
    {
        for (size_t i = ol_.step_left_speeds.size() - avg_count; i < ol_.step_left_speeds.size();
             i++)
        {
            left_ss += ol_.step_left_speeds[i];
            right_ss += ol_.step_right_speeds[i];
        }
        left_ss /= static_cast<float>(avg_count);
        right_ss /= static_cast<float>(avg_count);
    }

    // Steady-state mean of ACTUAL per-side voltage over the same tail used
    // for left_ss / right_ss. With heading-hold settled, this collapses to
    // ol_.current_voltage; if trim sits at a bias it carries through to the
    // regression honestly. Using the same avg_count window keeps voltage and
    // speed time-aligned.
    float left_v_ss = 0.0f, right_v_ss = 0.0f;
    if (avg_count > 0)
    {
        for (size_t i = ol_.step_left_volts.size() - avg_count; i < ol_.step_left_volts.size(); i++)
        {
            left_v_ss += ol_.step_left_volts[i];
            right_v_ss += ol_.step_right_volts[i];
        }
        left_v_ss /= static_cast<float>(avg_count);
        right_v_ss /= static_cast<float>(avg_count);
    }

    ol_.left_voltages.push_back(left_v_ss);
    ol_.left_speeds.push_back(left_ss);
    ol_.right_voltages.push_back(right_v_ss);
    ol_.right_speeds.push_back(right_ss);
    const float avg_speed   = (left_ss + right_ss) / 2.0f;
    const float avg_voltage = (left_v_ss + right_v_ss) / 2.0f;
    ol_.combined_voltages.push_back(avg_voltage);
    ol_.combined_speeds.push_back(avg_speed);

    // 63.2% rise time → coarse Tm sample
    if (avg_speed > MIN_SPEED_THRESHOLD && !ol_.step_left_speeds.empty())
    {
        const float target = avg_speed * 0.632f;
        for (size_t i = 1; i < ol_.step_left_speeds.size(); i++)
        {
            const float s_now = (ol_.step_left_speeds[i] + ol_.step_right_speeds[i]) / 2.0f;
            const float s_prev =
                (ol_.step_left_speeds[i - 1] + ol_.step_right_speeds[i - 1]) / 2.0f;
            if (s_prev < target && s_now >= target)
            {
                // i is index of post-SKIP sample, so loop_count of crossing =
                // SKIP_SAMPLES + (i - 1).
                ol_.tm_samples.push_back(loopsToSeconds(SKIP_SAMPLES + static_cast<int>(i) - 1));
                break;
            }
        }
    }

    // Advance to next voltage step.
    ol_.step_left_speeds.clear();
    ol_.step_right_speeds.clear();
    ol_.step_left_volts.clear();
    ol_.step_right_volts.clear();
    ol_.step_index++;
    ol_.current_voltage += ol_.step_voltage;
    if (ol_.current_voltage > ol_.max_voltage + 0.01f)
    {
        finishOpenLoop();
        return;
    }

    setVoltages(ol_.current_voltage, ol_.current_voltage);
    ol_.step_loop_count = 0;
    ol_.v_idx           = 0;
    ol_.v_count         = 0;
}

void DriverLab::finishOpenLoop()
{
    stopMotors();

    // Linear regression: speed = kM * voltage + intercept; kS = -intercept / kM.
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
        const float denom = n_out * sum_vv - sum_v * sum_v;
        if (std::fabs(denom) < 1e-12f)
        {
            kM_out = 0.0f;
            kS_out = 0.0f;
            return;
        }
        kM_out                = (n_out * sum_vs - sum_v * sum_s) / denom;
        const float intercept = (sum_s - kM_out * sum_v) / n_out;
        kS_out                = (std::fabs(kM_out) > 1e-6f) ? (-intercept / kM_out) : 0.0f;
    };

    constexpr float MIN_SPEED_THRESHOLD = 10.0f;

    float kM_calc = 0, kS_calc = 0;
    int   n_combined = 0;
    linRegress(ol_.combined_voltages, ol_.combined_speeds, MIN_SPEED_THRESHOLD, kM_calc, kS_calc,
               n_combined);

    float kM_L = 0, kS_L = 0, kM_R = 0, kS_R = 0;
    int   n_left = 0, n_right = 0;
    linRegress(ol_.left_voltages, ol_.left_speeds, MIN_SPEED_THRESHOLD, kM_L, kS_L, n_left);
    linRegress(ol_.right_voltages, ol_.right_speeds, MIN_SPEED_THRESHOLD, kM_R, kS_R, n_right);

    float tm_calc = 0.0f;
    if (!ol_.tm_samples.empty())
    {
        for (float t : ol_.tm_samples)
            tm_calc += t;
        tm_calc /= static_cast<float>(ol_.tm_samples.size());
    }

    // Per-motor calibration: only commit when regression succeeded (>= 2
    // valid points and a positive kM). A failed regression returns kM = 0,
    // which would otherwise propagate as kV = 1/0 = inf and corrupt every
    // dependent term.
    constexpr float MIN_VALID_KM = 1.0f;
    const bool      ol_left_ok   = (n_left >= 2) && (kM_L > MIN_VALID_KM);
    const bool      ol_right_ok  = (n_right >= 2) && (kM_R > MIN_VALID_KM);

    if (ol_left_ok)
    {
        settings_.kM_L = kM_L;
        settings_.kS_L = kS_L;
    }
    if (ol_right_ok)
    {
        settings_.kM_R = kM_R;
        settings_.kS_R = kS_R;
    }
    if (ol_left_ok && ol_right_ok)
    {
        settings_.kM = (kM_L + kM_R) / 2.0f;
        settings_.kS = (kS_L + kS_R) / 2.0f;
    }

    // Tm: only overwrite when at least one step actually crossed 63.2% within
    // its settle window. tm_samples empty → tm_calc = 0, which would zero
    // kA and drive kD negative through recalculateDerived. Preserve the
    // previous tm (from STEP or initDefaults) instead.
    if (!ol_.tm_samples.empty() && tm_calc > 0.0f)
        settings_.tm = tm_calc;

    settings_.recalculateDerived();

    printf("\n=== OL Results ===\n");
    printf("Combined: kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_calc, kS_calc, n_combined);
    printf("Left:     kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_L, kS_L, n_left);
    printf("Right:    kM=%.2f mm/s/V, kS=%.4f V (%d points)\n", kM_R, kS_R, n_right);
    printf("  -> kV = %.7f V/(mm/s)\n", settings_.kV);
    printf("  -> kA = %.7f V/(mm/s^2)\n", settings_.kA);
    printf("Yaw drift: %.2f deg\n\n", imu_->robot_angle());
    printBluetoothDiagnostics();
    printPrompt();

    trial_ = TrialState::Idle;
}

// ============================================================================
// STEP — forward step response, measures Tm
// ============================================================================

void DriverLab::startStepTrial(float volts, uint32_t duration_ms)
{
    printf("Step voltage: %.2f V, Duration: %lu ms\n", volts,
           static_cast<unsigned long>(duration_ms));

    step_.voltage        = volts;
    step_.duration_loops = static_cast<int>(msToLoops(duration_ms));
    step_.loop_count     = 0;
    step_.times_s.clear();
    step_.speeds.clear();
    step_.times_s.reserve(step_.duration_loops);
    step_.speeds.reserve(step_.duration_loops);

    beginCountdown("Step Response Trial", TrialState::Step);
}

void DriverLab::armStep()
{
    if (left_encoder_)
        left_encoder_->reset();
    if (right_encoder_)
        right_encoder_->reset();
    prev_left_ticks_   = 0;
    prev_right_ticks_  = 0;
    left_position_mm_  = 0.0f;
    right_position_mm_ = 0.0f;
    resetEncoderMA();

    // Trailing-window velocity state — reset at start so the first sample
    // doesn't compare against stale positions from a prior trial.
    step_.v_idx   = 0;
    step_.v_count = 0;

    reporter_.begin();
    reporter_.printStepHeader();

    setVoltages(step_.voltage, step_.voltage);
    trial_ = TrialState::Step;
}

void DriverLab::tickStep()
{
    if (step_.loop_count >= step_.duration_loops)
    {
        finishStep();
        return;
    }

    constexpr int SKIP_SAMPLES = 5; // 10 ms warm-up so the first VEL_WIN samples are valid

    // Trailing-window velocity from the oldest sample in the ring buffer.
    float win_left = 0.0f, win_right = 0.0f;
    if (step_.v_count > 0)
    {
        const int oldest = (step_.v_count < StepTrial::VEL_WIN) ? 0 : step_.v_idx;
        const int span = (step_.v_count < StepTrial::VEL_WIN) ? step_.v_count : StepTrial::VEL_WIN;
        const float dt_s = span * LOOP_INTERVAL_S;
        if (dt_s > 1e-6f)
        {
            win_left  = (left_position_mm_ - step_.left_pos_buf[oldest]) / dt_s;
            win_right = (right_position_mm_ - step_.right_pos_buf[oldest]) / dt_s;
        }
    }
    step_.left_pos_buf[step_.v_idx]  = left_position_mm_;
    step_.right_pos_buf[step_.v_idx] = right_position_mm_;
    step_.v_idx                      = (step_.v_idx + 1) % StepTrial::VEL_WIN;
    if (step_.v_count < StepTrial::VEL_WIN)
        step_.v_count++;

    const float speed = (win_left + win_right) / 2.0f;
    const float pos   = (left_position_mm_ + right_position_mm_) / 2.0f;

    // Internal Tm-fit array: dense, every loop tick after the warm-up window.
    if (step_.loop_count >= SKIP_SAMPLES)
    {
        step_.times_s.push_back(loopsToSeconds(step_.loop_count));
        step_.speeds.push_back(speed);
    }

    // CSV emission stays gated by the reporter — independent 10 ms cadence.
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (reporter_.isTimeToReport(now_ms))
        reporter_.reportStep(now_ms, step_.voltage, speed, pos);

    step_.loop_count++;
}

void DriverLab::finishStep()
{
    stopMotors();

    const int n = static_cast<int>(step_.speeds.size());
    if (n < 10)
    {
        printf("\nNot enough samples to calculate Tm\n");
        printBluetoothDiagnostics();
        printPrompt();
        trial_ = TrialState::Idle;
        return;
    }

    // Steady-state speed = average of last 20% of samples.
    const int tail_start  = n - n / 5;
    float     sum_final   = 0.0f;
    int       final_count = 0;
    for (int i = tail_start; i < n; i++)
    {
        sum_final += step_.speeds[i];
        final_count++;
    }
    const float v_final   = sum_final / final_count;
    const float threshold = v_final * 0.632f;

    float tm_calculated = 0.0f;
    bool  found         = false;
    for (int i = 0; i < n; i++)
    {
        if (step_.speeds[i] >= threshold)
        {
            if (i > 0 && step_.speeds[i - 1] < threshold)
            {
                const float frac =
                    (threshold - step_.speeds[i - 1]) / (step_.speeds[i] - step_.speeds[i - 1]);
                tm_calculated =
                    step_.times_s[i - 1] + frac * (step_.times_s[i] - step_.times_s[i - 1]);
            }
            else
            {
                tm_calculated = step_.times_s[i];
            }
            found = true;
            break;
        }
    }

    printf("\n=== Step Response Results ===\n");
    printf("Steady-state speed: %.1f mm/s\n", v_final);
    printf("63.2%% threshold:    %.1f mm/s\n", threshold);

    if (found && tm_calculated > 0.001f && tm_calculated < 2.0f)
    {
        settings_.tm = tm_calculated;
        settings_.td = tm_calculated; // Jurababa: Td = Tm uniformly (see tuning.h:52-59)
        settings_.recalculateDerived();
        printf("Tm = %.5f s  (motor time constant)\n", settings_.tm);
        printf("  -> kA = %.7f V/(mm/s^2)\n", settings_.kA);
        printf("  -> Td = %.5f s  (= Tm)\n", settings_.td);
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

    printf("Samples: %d\n\n", n);
    printBluetoothDiagnostics();
    printPrompt();
    trial_ = TrialState::Idle;
}

// ============================================================================
// MOVE — closed-loop forward profile (own profile + own forward PD)
// ============================================================================

void DriverLab::startMoveTrial(float distance, float speed, float accel, int mode)
{
    printf("Distance: %.1f mm, Speed: %.1f mm/s, Accel: %.1f mm/s^2\n", distance, speed, accel);
    const char* mode_label = mode == 0   ? "FULL_CONTROL (FF+PD)"
                             : mode == 1 ? "NO_FF (PD only)"
                             : mode == 2 ? "ONLY_FF"
                                         : "(unknown)";
    printf("Mode: %d (%s)\n", mode, mode_label);

    move_.distance_mm     = distance;
    move_.top_speed       = speed;
    move_.acceleration    = accel;
    move_.mode            = mode;
    move_.max_speed_error = 0.0f;
    move_.sum_speed_error = 0.0f;
    move_.diag_count      = 0;
    move_.max_pos_error   = 0.0f;
    move_.max_volts       = 0.0f;
    move_.loop_count      = 0;

    beginCountdown("Move Trial", TrialState::Move);
}

void DriverLab::armMove()
{
    if (left_encoder_)
        left_encoder_->reset();
    if (right_encoder_)
        right_encoder_->reset();
    prev_left_ticks_   = 0;
    prev_right_ticks_  = 0;
    left_position_mm_  = 0.0f;
    right_position_mm_ = 0.0f;
    resetEncoderMA();

    forward_pid_.setGains(settings_.kP, settings_.kD);
    forward_pid_.reset();

    forward_profile_.reset();
    forward_profile_.start(move_.distance_mm, move_.top_speed, move_.acceleration);

    reporter_.begin();
    // MotorLab-style controller header: FF / CTRL / Motor voltages reported
    // separately so the dashboard can plot each contribution. Stereo
    // extension: per-wheel FF (asymmetric kV/kS shows up here) plus a single
    // shared ctrl_v (PID is computed once and added equally to both sides).
    // set_pos / actual_pos are logged for offline tracking-error inspection
    // but not plotted by default — same as MotorLab.
    printf("time_ms,set_pos,actual_pos,set_speed,actual_speed,ff_v_left,ff_v_right,ctrl_v,"
           "total_v_left,total_v_right,left_speed,right_speed\n");

    trial_ = TrialState::Move;
}

void DriverLab::tickMove()
{
    forward_profile_.update(LOOP_INTERVAL_S);

    const float set_speed = forward_profile_.speed();
    const float fwd_change_mm =
        0.5f * (left_velocity_mmps_ + right_velocity_mmps_) * LOOP_INTERVAL_S;

    // Motorlab numbering: 0 = FULL_CONTROL (FF+PD), 1 = NO_FF (PD only),
    // 2 = ONLY_FF. Disable PD only when ONLY_FF; disable FF only when NO_FF.
    const float pid_output =
        (move_.mode != 2) ? forward_pid_.update(set_speed, fwd_change_mm) : 0.0f;

    float ff_left  = 0.0f;
    float ff_right = 0.0f;
    if (move_.mode != 1)
    {
        ff_left  = feedforwardVolts(set_speed, forward_profile_.acceleration(), /*left=*/true);
        ff_right = feedforwardVolts(set_speed, forward_profile_.acceleration(), /*left=*/false);
    }

    const float left_v  = ff_left + pid_output;
    const float right_v = ff_right + pid_output;
    setVoltages(left_v, right_v);

    // Diagnostics
    const float actual_speed = (left_velocity_mmps_ + right_velocity_mmps_) / 2.0f;
    const float speed_err    = std::fabs(set_speed - actual_speed);
    move_.sum_speed_error += speed_err;
    move_.diag_count++;
    if (speed_err > move_.max_speed_error)
        move_.max_speed_error = speed_err;
    const float pos_error = forward_pid_.error();
    if (std::fabs(pos_error) > move_.max_pos_error)
        move_.max_pos_error = std::fabs(pos_error);
    const float abs_volts = std::max(std::fabs(left_v), std::fabs(right_v));
    if (abs_volts > move_.max_volts)
        move_.max_volts = abs_volts;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (reporter_.isTimeToReport(now_ms))
    {
        const uint32_t elapsed    = now_ms - reporter_.startTime();
        const float    set_pos    = forward_profile_.position();
        const float    actual_pos = (left_position_mm_ + right_position_mm_) * 0.5f;
        printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
               static_cast<unsigned long>(elapsed), set_pos, actual_pos, set_speed, actual_speed,
               ff_left, ff_right, pid_output, left_v, right_v, left_velocity_mmps_,
               right_velocity_mmps_);
        reporter_.incrementSampleCount();
    }

    move_.loop_count++;
    if (forward_profile_.finished())
    {
        finishMove();
    }
}

void DriverLab::finishMove()
{
    stopMotors();

    const float actual_dist     = (left_position_mm_ + right_position_mm_) / 2.0f;
    const float final_pos_error = move_.distance_mm - actual_dist;
    const float avg_speed_error =
        (move_.diag_count > 0) ? (move_.sum_speed_error / move_.diag_count) : 0.0f;

    printf("\n=== Move Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("\n--- Diagnostics ---\n");
    printf("  Final position error: %+.2f mm\n", final_pos_error);
    printf("  Max position error:   %.2f mm\n", move_.max_pos_error);
    printf("  Max speed error:      %.1f mm/s\n", move_.max_speed_error);
    printf("  Avg speed error:      %.1f mm/s\n", avg_speed_error);
    printf("  Max motor volts:      %.2f V\n", move_.max_volts);
    printf("\n--- Tuning Tips ---\n");
    if (move_.mode == 0 || move_.mode == 2)
    {
        if (avg_speed_error > 20.0f)
            printf("  ! Steady-state error high: kV may be wrong (run OL to recalibrate)\n");
        if (move_.max_speed_error > move_.top_speed * 0.3f)
            printf("  ! Large transient error: increase kA or reduce acceleration\n");
    }
    if (move_.mode == 1 || move_.mode == 2)
    {
        if (move_.max_volts > MAX_VOLTAGE * 0.8f)
            printf("  ! Near voltage saturation: reduce kP or lower speed/accel\n");
        if (move_.max_pos_error > 10.0f)
            printf("  ! Position drift > 10mm: increase kP\n");
    }
    if (std::fabs(final_pos_error) > 5.0f)
        printf("  ! Stopping error > 5mm: check deceleration phase\n");
    if (move_.max_speed_error < move_.top_speed * 0.1f && std::fabs(final_pos_error) < 3.0f)
        printf("  Looks good! Speed tracking within 10%%, position error < 3mm.\n");
    printf("\n");
    printBluetoothDiagnostics();
    printPrompt();
    trial_ = TrialState::Idle;
}

// ============================================================================
// TURN — closed-loop rotation profile (own profile + own rotation PD)
// ============================================================================

void DriverLab::startTurnTrial(float degrees, float omega, float alpha)
{
    printf("Angle: %.1f deg, Speed: %.1f deg/s, Accel: %.1f deg/s^2\n", degrees, omega, alpha);
    printf("TURN_KP: %.4f, TURN_KD: %.4f\n", settings_.turnKP, settings_.turnKD);

    turn_.degrees            = degrees;
    turn_.top_omega          = omega;
    turn_.alpha              = alpha;
    turn_.peak_overshoot_deg = 0.0f;
    turn_.max_yaw_error      = 0.0f;
    turn_.max_volts          = 0.0f;
    turn_.past_target        = false;
    turn_.loop_count         = 0;

    beginCountdown("Turn Trial", TrialState::Turn);
}

void DriverLab::armTurn()
{
    rotation_pid_.setGains(settings_.turnKP, settings_.turnKD);
    rotation_pid_.reset();

    rotation_profile_.reset();
    rotation_profile_.start(turn_.degrees, std::fabs(turn_.top_omega), std::fabs(turn_.alpha));

    imu_->reset(); // zero yaw reference for this trial

    reporter_.begin();
    // TURN has no rotation feedforward — last_rotation_volts is purely the
    // PID output (see tickTurn). So unlike MOVE we emit only ctrl_v plus the
    // per-wheel applied voltages. If a rotation FF term is ever added,
    // expand this header to include ff_v.
    printf("time_ms,set_omega,actual_yaw,actual_omega,error,ctrl_v,total_v_left,total_v_right\n");

    trial_ = TrialState::Turn;
}

void DriverLab::tickTurn()
{
    // Rotation profile is integrated at 500 Hz like forward; the rotation PD,
    // however, only fires when the IMU has a fresh packet (~100 Hz). Between
    // packets the controller's last output is held (zero-order hold).
    rotation_profile_.update(LOOP_INTERVAL_S);
    const float set_omega        = rotation_profile_.speed();
    const float profile_dir      = (turn_.degrees >= 0.0f) ? 1.0f : -1.0f;
    const float signed_set_omega = profile_dir * set_omega;

    static float last_rotation_volts = 0.0f;

    if (imu_->has_new_yaw_sample())
    {
        const float rot_change = imu_->robot_rot_change(); // per-PACKET delta (deg)
        last_rotation_volts    = rotation_pid_.update(signed_set_omega, rot_change);
    }
    setVoltages(-last_rotation_volts, +last_rotation_volts);

    // Diagnostics
    const float actual_yaw   = imu_->robot_angle();
    const float actual_omega = imu_->robot_omega();
    const float rot_error    = rotation_pid_.error();

    const float abs_error = std::fabs(rot_error);
    if (abs_error > turn_.max_yaw_error)
        turn_.max_yaw_error = abs_error;
    const float abs_volts = std::fabs(last_rotation_volts);
    if (abs_volts > turn_.max_volts)
        turn_.max_volts = abs_volts;

    if (std::fabs(actual_yaw) >= std::fabs(turn_.degrees) * 0.95f)
        turn_.past_target = true;
    if (turn_.past_target)
    {
        const float overshoot = std::fabs(actual_yaw) - std::fabs(turn_.degrees);
        if (overshoot > turn_.peak_overshoot_deg)
            turn_.peak_overshoot_deg = overshoot;
    }

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (reporter_.isTimeToReport(now_ms))
    {
        const uint32_t elapsed = now_ms - reporter_.startTime();
        printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n", static_cast<unsigned long>(elapsed),
               signed_set_omega, actual_yaw, actual_omega, rot_error, last_rotation_volts,
               -last_rotation_volts, +last_rotation_volts);
        reporter_.incrementSampleCount();
    }

    turn_.loop_count++;
    if (rotation_profile_.finished())
    {
        finishTurn();
    }
}

void DriverLab::finishTurn()
{
    stopMotors();

    const float final_error = turn_.degrees - imu_->robot_angle();

    printf("\n=== Turn Trial Complete ===\n");
    printf("Samples: %lu\n", static_cast<unsigned long>(reporter_.sampleCount()));
    printf("\n--- Diagnostics ---\n");
    printf("  Final error:     %+.2f deg\n", final_error);
    printf("  Peak overshoot:  %.2f deg (%.1f%%)\n", turn_.peak_overshoot_deg,
           (std::fabs(turn_.degrees) > 0.1f)
               ? (turn_.peak_overshoot_deg / std::fabs(turn_.degrees) * 100.0f)
               : 0.0f);
    printf("  Max yaw error:   %.2f deg\n", turn_.max_yaw_error);
    printf("  Max motor volts: %.2f V\n", turn_.max_volts);
    printf("\n--- Tuning Tips ---\n");
    if (turn_.peak_overshoot_deg > std::fabs(turn_.degrees) * 0.10f)
        printf("  ! Overshoot > 10%%: increase turnKD or decrease turnKP\n");
    if (std::fabs(final_error) > 3.0f)
        printf("  ! Final error > 3 deg: increase turnKP\n");
    if (turn_.max_volts > MAX_VOLTAGE * 0.9f)
        printf("  ! Near voltage saturation: reduce speed or lower turnKP\n");
    if (turn_.peak_overshoot_deg < 1.0f && std::fabs(final_error) < 2.0f &&
        turn_.max_volts < MAX_VOLTAGE * 0.7f)
        printf("  Looks good! Gains are well-tuned for this speed.\n");
    printf("\n");
    printBluetoothDiagnostics();
    printPrompt();
    trial_ = TrialState::Idle;
}

// ============================================================================
// TURN-OL — open-loop differential-voltage sweep, measures rotational kM
// ============================================================================

void DriverLab::startTurnOpenLoopTrial(float max_v, float step_v, uint32_t settle_ms)
{
    printf("Max diff: %.2f V, Step: %.2f V, Settle: %lu ms\n", max_v, step_v,
           static_cast<unsigned long>(settle_ms));

    tol_.max_voltage     = max_v;
    tol_.step_voltage    = step_v;
    tol_.settle_loops    = msToLoops(settle_ms);
    tol_.current_voltage = step_v;
    tol_.step_loop_count = 0;
    tol_.step_index      = 0;
    // yaw_at_step_start is set in armTurnOpenLoop after the IMU reset.
    tol_.yaw_at_step_start = 0.0f;
    tol_.step_omegas.clear();
    tol_.step_times_s.clear();
    tol_.diff_voltages.clear();
    tol_.omega_steady.clear();
    tol_.rot_tm_samples.clear();
    tol_.pausing          = false;
    tol_.pause_loop_count = 0;

    beginCountdown("TURN Open Loop Voltage Sweep", TrialState::TurnOpenLoop);
}

void DriverLab::armTurnOpenLoop()
{
    imu_->reset(); // zero yaw for omega integration

    reporter_.begin();
    reporter_.printTurnOpenLoopHeader();

    tol_.yaw_at_step_start = imu_->robot_angle();

    setVoltages(-tol_.current_voltage, +tol_.current_voltage);
    trial_ = TrialState::TurnOpenLoop;
}

void DriverLab::tickTurnOpenLoop()
{
    constexpr int   SKIP_SAMPLES        = 20; // 40 ms transient skip
    constexpr float MIN_OMEGA_THRESHOLD = 5.0f;
    // 200 ms inter-step pause so the body stops spinning before the next
    // diff voltage arms.
    const int pause_loops = static_cast<int>(msToLoops(200));

    if (tol_.pausing)
    {
        tol_.pause_loop_count++;
        if (tol_.pause_loop_count < pause_loops)
            return;

        // Pause complete — advance to next step or finish.
        tol_.pausing          = false;
        tol_.pause_loop_count = 0;
        tol_.current_voltage += tol_.step_voltage;
        if (tol_.current_voltage > tol_.max_voltage + 0.01f)
        {
            finishTurnOpenLoop();
            return;
        }
        tol_.step_loop_count = 0;
        tol_.step_index++;
        tol_.yaw_at_step_start = imu_->robot_angle();
        tol_.step_omegas.clear();
        tol_.step_times_s.clear();
        setVoltages(-tol_.current_voltage, +tol_.current_voltage);
        return;
    }

    // ISR-cached omega — held flat between BNO085 packets. See drivers/imu.cpp
    // and config/sensors.h IMU_PACKET_HZ comment for why this beats sampling
    // at the loop rate.
    const float cur_yaw = imu_->robot_angle();
    const float omega   = imu_->robot_omega();

    if (tol_.step_loop_count >= SKIP_SAMPLES)
    {
        tol_.step_omegas.push_back(omega);
        tol_.step_times_s.push_back(loopsToSeconds(tol_.step_loop_count));

        const int kept = tol_.step_loop_count - SKIP_SAMPLES;
        if (kept % 10 == 0)
        {
            const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            reporter_.reportTurnOpenLoop(now_ms, tol_.current_voltage, cur_yaw, omega);
        }
    }

    tol_.step_loop_count++;
    if (static_cast<uint32_t>(tol_.step_loop_count) < tol_.settle_loops)
        return;

    // ------------------------ end of step ------------------------
    // Average tail = steady-state omega.
    const size_t avg_count = std::min(tol_.step_omegas.size(), size_t(20));
    float        omega_ss  = 0.0f;
    if (avg_count > 0)
    {
        for (size_t i = tol_.step_omegas.size() - avg_count; i < tol_.step_omegas.size(); i++)
            omega_ss += tol_.step_omegas[i];
        omega_ss /= static_cast<float>(avg_count);
    }

    tol_.diff_voltages.push_back(tol_.current_voltage);
    tol_.omega_steady.push_back(omega_ss);

    // 63.2% rise time as coarse rot_tm sample.
    if (std::fabs(omega_ss) > MIN_OMEGA_THRESHOLD && tol_.step_omegas.size() > 1)
    {
        const float target = omega_ss * 0.632f;
        for (size_t i = 1; i < tol_.step_omegas.size(); i++)
        {
            const bool crossed =
                (omega_ss > 0 && tol_.step_omegas[i - 1] < target &&
                 tol_.step_omegas[i] >= target) ||
                (omega_ss < 0 && tol_.step_omegas[i - 1] > target && tol_.step_omegas[i] <= target);
            if (crossed)
            {
                tol_.rot_tm_samples.push_back(tol_.step_times_s[i - 1]);
                break;
            }
        }
    }

    // Inter-step pause: stop motors and dwell so the body de-spins.
    stopMotors();
    tol_.pausing = true;
}

void DriverLab::finishTurnOpenLoop()
{
    stopMotors();

    auto linRegress = [](const std::vector<float>& v, const std::vector<float>& w, float min_omega,
                         float& kM_out, int& n_out)
    {
        float sum_v = 0, sum_w = 0, sum_vw = 0, sum_vv = 0;
        n_out = 0;
        for (size_t i = 0; i < v.size(); i++)
        {
            if (std::fabs(w[i]) > min_omega)
            {
                sum_v += v[i];
                sum_w += w[i];
                sum_vw += v[i] * w[i];
                sum_vv += v[i] * v[i];
                n_out++;
            }
        }
        if (n_out < 2)
        {
            kM_out = 0.0f;
            return;
        }
        const float denom = n_out * sum_vv - sum_v * sum_v;
        if (std::fabs(denom) < 1e-12f)
        {
            kM_out = 0.0f;
            return;
        }
        kM_out = (n_out * sum_vw - sum_v * sum_w) / denom;
    };

    constexpr float MIN_OMEGA_THRESHOLD = 5.0f;
    float           rot_kM              = 0.0f;
    int             n_pts               = 0;
    linRegress(tol_.diff_voltages, tol_.omega_steady, MIN_OMEGA_THRESHOLD, rot_kM, n_pts);

    float rot_tm_calc = 0.0f;
    if (!tol_.rot_tm_samples.empty())
    {
        for (float t : tol_.rot_tm_samples)
            rot_tm_calc += t;
        rot_tm_calc /= static_cast<float>(tol_.rot_tm_samples.size());
    }

    constexpr float MIN_VALID_ROT_KM = 10.0f;
    const bool      ol_ok            = (n_pts >= 2) && (rot_kM > MIN_VALID_ROT_KM);

    if (ol_ok)
    {
        settings_.rot_kM = rot_kM;
        if (rot_tm_calc > 0.001f && rot_tm_calc < 1.0f)
            settings_.rot_tm = rot_tm_calc;
        settings_.recalculateRotation();
    }

    printf("\n=== TURN-OL Results ===\n");
    printf("ROT_KM = %.2f deg/s/V (%d points)\n", rot_kM, n_pts);
    if (rot_tm_calc > 0.0f)
        printf("ROT_TM (coarse) = %.5f s  (run TURN-STEP for a cleaner value)\n", rot_tm_calc);
    if (ol_ok)
    {
        printf("  -> turnKP = %.5f, turnKD = %.5f  (recomputed)\n", settings_.turnKP,
               settings_.turnKD);
    }
    else
    {
        printf("  ! Regression failed -- keeping previous rot_kM/turnKP/turnKD\n");
    }
    printBluetoothDiagnostics();
    printPrompt();
    trial_ = TrialState::Idle;
}

// ============================================================================
// TURN-STEP — differential-voltage step response, measures rotational ROT_TM
// ============================================================================

void DriverLab::startTurnStepTrial(float diff_v, uint32_t duration_ms)
{
    printf("Diff voltage: %.2f V, Duration: %lu ms\n", diff_v,
           static_cast<unsigned long>(duration_ms));

    tstep_.diff_voltage   = diff_v;
    tstep_.duration_loops = static_cast<int>(msToLoops(duration_ms));
    tstep_.loop_count     = 0;
    tstep_.times_s.clear();
    tstep_.omegas.clear();
    // Internal samples push at IMU packet rate (~100 Hz), not loop rate.
    // Reserve for one sample per ms / IMU_PACKET_HZ = duration_ms / 10.
    const size_t imu_samples = static_cast<size_t>(duration_ms / 10) + 4;
    tstep_.times_s.reserve(imu_samples);
    tstep_.omegas.reserve(imu_samples);

    beginCountdown("TURN Step Response", TrialState::TurnStep);
}

void DriverLab::armTurnStep()
{
    imu_->reset();

    // Seed packet-edge gate so the first tick doesn't push a stale packet
    // from before the trial armed.
    tstep_.last_packet_seq = imu_->packet_seq();

    reporter_.begin();
    reporter_.printTurnStepHeader();

    setVoltages(-tstep_.diff_voltage, +tstep_.diff_voltage);
    trial_ = TrialState::TurnStep;
}

void DriverLab::tickTurnStep()
{
    if (tstep_.loop_count >= tstep_.duration_loops)
    {
        finishTurnStep();
        return;
    }

    const float cur_yaw = imu_->robot_angle();
    const float omega   = imu_->robot_omega();

    // Internal Tm-fit array: one sample per IMU packet edge. omega is held
    // flat between packets, so loop-rate sampling would just repeat values.
    const uint32_t seq_now = imu_->packet_seq();
    if (seq_now != tstep_.last_packet_seq)
    {
        tstep_.times_s.push_back(loopsToSeconds(tstep_.loop_count));
        tstep_.omegas.push_back(omega);
        tstep_.last_packet_seq = seq_now;
    }

    // CSV emission stays gated by the reporter — independent 10 ms cadence.
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (reporter_.isTimeToReport(now_ms))
        reporter_.reportTurnStep(now_ms, tstep_.diff_voltage, cur_yaw, omega);

    tstep_.loop_count++;
}

void DriverLab::finishTurnStep()
{
    stopMotors();

    const int n = static_cast<int>(tstep_.omegas.size());
    if (n < 10)
    {
        printf("\nNot enough samples to compute ROT_TM\n");
        printBluetoothDiagnostics();
        printPrompt();
        trial_ = TrialState::Idle;
        return;
    }

    const int tail_start = n - n / 5;
    float     sum_ss     = 0.0f;
    int       ss_count   = 0;
    for (int i = tail_start; i < n; i++)
    {
        sum_ss += tstep_.omegas[i];
        ss_count++;
    }
    const float omega_ss = sum_ss / ss_count;

    if (std::fabs(omega_ss) < 5.0f)
    {
        printf("\nSteady-state omega too small (%.2f deg/s) -- increase diff voltage.\n", omega_ss);
        printBluetoothDiagnostics();
        printPrompt();
        trial_ = TrialState::Idle;
        return;
    }

    const float threshold   = omega_ss * 0.632f;
    float       rot_tm_calc = 0.0f;
    bool        found       = false;
    for (int i = 1; i < n; i++)
    {
        const bool crossed =
            (omega_ss > 0) ? (tstep_.omegas[i - 1] < threshold && tstep_.omegas[i] >= threshold)
                           : (tstep_.omegas[i - 1] > threshold && tstep_.omegas[i] <= threshold);
        if (crossed)
        {
            const float frac =
                (threshold - tstep_.omegas[i - 1]) / (tstep_.omegas[i] - tstep_.omegas[i - 1]);
            rot_tm_calc =
                tstep_.times_s[i - 1] + frac * (tstep_.times_s[i] - tstep_.times_s[i - 1]);
            found = true;
            break;
        }
    }

    printf("\n=== TURN-STEP Results ===\n");
    printf("Steady-state omega: %.1f deg/s\n", omega_ss);
    printf("63.2%% threshold:   %.1f deg/s\n", threshold);

    if (found && rot_tm_calc > 0.001f && rot_tm_calc < 1.0f)
    {
        settings_.rot_tm = rot_tm_calc;
        settings_.rot_td = rot_tm_calc; // Jurababa: Td = Tm uniformly (see tuning.h:52-59)
        settings_.recalculateRotation();
        printf("ROT_TM = %.5f s\n", settings_.rot_tm);
        printf("  -> ROT_TD = %.5f s\n", settings_.rot_td);
        printf("  -> turnKP = %.5f, turnKD = %.5f  (recomputed)\n", settings_.turnKP,
               settings_.turnKD);
    }
    else
    {
        printf("ROT_TM: could not determine (omega may not have settled)\n");
    }

    printf("Samples: %d\n\n", n);
    printBluetoothDiagnostics();
    printPrompt();
    trial_ = TrialState::Idle;
}

void DriverLab::printBluetoothDiagnostics()
{
    const BluetoothStdio::Diagnostics bt = BluetoothStdio::diagnostics();
    if (bt.dropped_bytes == 0 && bt.suppressed_csv_lines == 0 && bt.max_depth < bt.ring_size / 2)
        return;

    printf("--- Bluetooth TX ---\n");
    printf("  CSV interval:      %lu ms\n", static_cast<unsigned long>(bt.csv_interval_ms));
    printf("  Ring depth/max:    %lu/%lu bytes\n", static_cast<unsigned long>(bt.depth),
           static_cast<unsigned long>(bt.max_depth));
    printf("  Suppressed rows:   %lu\n", static_cast<unsigned long>(bt.suppressed_csv_lines));
    printf("  Dropped bytes:     %lu\n\n", static_cast<unsigned long>(bt.dropped_bytes));
}

// ============================================================================
// CLI Processing
// ============================================================================

bool DriverLab::processSerial()
{
    const int result = readSerialLine();
    if (result == 1)
    {
        DriverLabArgs args = tokenize();
        if (args.argc > 0)
        {
            executeCommand(args);
        }
        clearInput();
        // Trial-active commands (OL, STEP, MOVE, TURN, ...) move trial_ out
        // of Idle (into Countdown) and the matching finish*() prints the
        // prompt themselves on completion. Skip the prompt here in that
        // case so "DRIVERLAB> " doesn't appear mid-countdown.
        if (trial_ == TrialState::Idle)
            printPrompt();
        return true;
    }
    return false;
}

int DriverLab::readSerialLine()
{
    while (true)
    {
        int  c         = getchar_timeout_us(0); // USB
        bool from_uart = false;

        if (c == PICO_ERROR_TIMEOUT && uart_is_readable(uart0))
        {
            c         = uart_getc(uart0);
            from_uart = true;
        }

        if (c == PICO_ERROR_TIMEOUT)
            return 0;

        char ch = static_cast<char>(c);

        if (ch == '\n' || ch == '\r')
        {
            if (echo_enabled_)
                printf("\n");
            if (input_index_ > 0)
                saveToHistory();
            history_nav_idx_ = -1;
            return 1;
        }

        if (ch == '\b' || ch == 127)
        {
            if (input_index_ > 0)
            {
                input_index_--;
                input_buffer_[input_index_] = '\0';
                if (echo_enabled_)
                    printf("\b \b");
            }
            continue;
        }

        if (isprint(ch))
        {
            ch = static_cast<char>(toupper(ch));
            if (echo_enabled_)
                putchar(ch);
            if (input_index_ < DRIVERLAB_INPUT_BUFFER_SIZE - 1)
            {
                input_buffer_[input_index_++] = ch;
                input_buffer_[input_index_]   = '\0';
            }
        }
    }
}

void DriverLab::executeLine(const char* line)
{
    if (line == nullptr)
        return;

    size_t i = 0;
    while (line[i] != '\0' && line[i] != '\n' && line[i] != '\r' &&
           i < DRIVERLAB_INPUT_BUFFER_SIZE - 1)
    {
        input_buffer_[i] = line[i];
        ++i;
    }
    input_buffer_[i] = '\0';
    input_index_     = static_cast<int>(i);

    DriverLabArgs args = tokenize();
    if (args.argc > 0)
        executeCommand(args);

    clearInput();
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

    if (strcmp(cmd, "?") == 0 || strcmp(cmd, "HELP") == 0)
        cmdHelp();
    else if (strcmp(cmd, "ID") == 0)
        cmdId();
    else if (strcmp(cmd, "SETTINGS") == 0 || strcmp(cmd, "S") == 0)
        cmdSettings();
    else if (strcmp(cmd, "INIT") == 0)
        cmdInitSettings();
    else if (strcmp(cmd, "KM") == 0)
        cmdSetKm(args);
    else if (strcmp(cmd, "TM") == 0)
        cmdSetTm(args);
    else if (strcmp(cmd, "ZETA") == 0)
        cmdSetZeta(args);
    else if (strcmp(cmd, "TD") == 0)
        cmdSetTd(args);
    else if (strcmp(cmd, "KP") == 0)
        cmdSetKp(args);
    else if (strcmp(cmd, "KD") == 0)
        cmdSetKd(args);
    else if (strcmp(cmd, "BIAS") == 0)
        cmdSetBiasFF(args);
    else if (strcmp(cmd, "SPEEDFF") == 0)
        cmdSetSpeedFF(args);
    else if (strcmp(cmd, "ACCFF") == 0)
        cmdSetAccFF(args);
    else if (strcmp(cmd, "BATTERY") == 0 || strcmp(cmd, "BAT") == 0)
        cmdBattery();
    else if (strcmp(cmd, "ENCODERS") == 0 || strcmp(cmd, "ENC") == 0)
        cmdEncoders();
    else if (strcmp(cmd, "YAW") == 0 || strcmp(cmd, "IMU") == 0)
        cmdYaw();
    else if (strcmp(cmd, "YAWVEL") == 0 || strcmp(cmd, "IMUVEL") == 0)
        cmdYawVel();
    else if (strcmp(cmd, "YAWCON") == 0 || strcmp(cmd, "IMUCON") == 0)
        cmdYawContinuous(args);
    else if (strcmp(cmd, "YAWRESET") == 0 || strcmp(cmd, "IMURESET") == 0)
        cmdYawReset();
    else if (strcmp(cmd, "LTOF") == 0)
        cmdLeftTof();
    else if (strcmp(cmd, "FTOF") == 0)
        cmdFrontTof();
    else if (strcmp(cmd, "RTOF") == 0)
        cmdRightTof();
    else if (strcmp(cmd, "TOFCON") == 0)
        cmdTofContinuous(args);
    else if (strcmp(cmd, "LENC") == 0)
        cmdLeftEncoder();
    else if (strcmp(cmd, "RENC") == 0)
        cmdRightEncoder();
    else if (strcmp(cmd, "ENCRESET") == 0)
        cmdEncoderReset();
    else if (strcmp(cmd, "ENCCON") == 0)
        cmdEncoderContinuous(args);
    else if (strcmp(cmd, "OPENLOOP") == 0 || strcmp(cmd, "OL") == 0)
        cmdOpenLoop(args);
    else if (strcmp(cmd, "STEP") == 0)
        cmdStep(args);
    else if (strcmp(cmd, "MOVE") == 0)
        cmdMove(args);
    else if (strcmp(cmd, "TURN") == 0)
        cmdTurn(args);
    else if (strcmp(cmd, "TURNOL") == 0 || strcmp(cmd, "TOL") == 0)
        cmdTurnOpenLoop(args);
    else if (strcmp(cmd, "TURNSTEP") == 0 || strcmp(cmd, "TSTEP") == 0)
        cmdTurnStep(args);
    else if (strcmp(cmd, "ROT_ZETA") == 0)
        cmdSetRotZeta(args);
    else if (strcmp(cmd, "ROT_TD") == 0)
        cmdSetRotTd(args);
    else if (strcmp(cmd, "TURN_KP") == 0)
        cmdSetTurnKp(args);
    else if (strcmp(cmd, "TURN_KD") == 0)
        cmdSetTurnKd(args);
    else if (strcmp(cmd, "VOLTS") == 0 || strcmp(cmd, "V") == 0)
        cmdVoltage(args);
    else if (strcmp(cmd, "VL") == 0)
        cmdVoltageLeft(args);
    else if (strcmp(cmd, "VR") == 0)
        cmdVoltageRight(args);
    else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "X") == 0)
        cmdStop();
    else if (strcmp(cmd, "BTLOG") == 0)
        cmdBtLog(args);
    else if (strcmp(cmd, "EXPORT") == 0)
        cmdExport();
    else if (strcmp(cmd, "GPIO") == 0)
        cmdGpioDiag(args);
    else if (strcmp(cmd, "DIRTEST") == 0)
        cmdDirTest(args);
    else if (strcmp(cmd, "LINPOS") == 0)
        cmdLinePosition();
    else if (strcmp(cmd, "LININTER") == 0)
        cmdLineIntersection();
    else if (strcmp(cmd, "LINCON") == 0)
        cmdLineContinuous(args);
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
    else if (strcmp(cmd, "R") == 0)
        cmdRepeat();
    else if (strcmp(cmd, "H") == 0)
        cmdHistory();
    else if (cmd[0] >= '1' && cmd[0] <= '9' && strlen(cmd) <= 2)
        executeHistorySelection(atoi(cmd));
    else
        printf("Unknown command: %s (type '?' for help)\n", cmd);
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
        return false;

    const float val = static_cast<float>(atof(args.argv[index]));
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
    printf("  MOVE [mm mm/s mm/s^2 mode]      Forward motion   (default: 480 200 500 0)\n");
    printf("    mode: 0=FULL_CONTROL (FF+PD)  1=NO_FF (PD only, stress)  2=ONLY_FF\n");
    printf("       mode: 0=FF only, 1=PD only, 2=FF+PD\n");
    printf("  TURN [deg deg/s deg/s^2]        Turn in place    (default: 90 360 720)\n");
    printf("       +deg=CCW(left), -deg=CW(right)\n");
    printf("  TURNOL  [max step settle_ms]    Rotation OL sweep  (default: 3 0.5 800)\n");
    printf("  TURNSTEP [diff_v duration_ms]   Rotation step      (default: 1.5 1000)\n");
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
    printf("  BTLOG [ms|RESET]    Bluetooth CSV interval/diagnostics (default: 50 ms)\n");
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
        // Per-motor calibration is preserved on a READ -> WRITE round-trip
        // (e.g. dashboard echoing the same combined value back). Only
        // propagate to per-motor when the value actually changes.
        const bool propagate = std::fabs(val - settings_.kM) > 1e-3f;
        settings_.kM         = val;
        if (propagate)
        {
            settings_.kM_L = val;
            settings_.kM_R = val;
        }
        settings_.recalculateDerived();
        printf("kM = %.2f (%s)\n", settings_.kM,
               propagate ? "kM_L=kM_R propagated, derived updated"
                         : "per-motor preserved, derived updated");
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
        settings_.td = val / 2.0f;
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
        settings_.recalculatePD();
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
        settings_.recalculatePD();
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
        const bool propagate = std::fabs(val - settings_.kS) > 1e-6f;
        settings_.kS         = val;
        if (propagate)
        {
            settings_.kS_L = val;
            settings_.kS_R = val;
        }
        printf("kS = %.5f V (%s)\n", settings_.kS,
               propagate ? "kS_L=kS_R propagated" : "per-motor preserved");
    }
    else if (args.argc == 1)
    {
        printf("kS = %.5f V\n", settings_.kS);
    }
}

void DriverLab::cmdSetSpeedFF(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 0.1f, val))
    {
        const bool propagate = std::fabs(val - settings_.kV) > 1e-6f;
        if (val > 1e-9f)
        {
            settings_.kM = 1.0f / val;
            if (propagate)
            {
                settings_.kM_L = settings_.kM;
                settings_.kM_R = settings_.kM;
            }
            settings_.recalculateFeedforward();
        }
        else
        {
            settings_.kV = val;
        }
        printf("kV = %.7f V/(mm/s) (kM=%.2f, kA=%.7f, %s)\n", settings_.kV, settings_.kM,
               settings_.kA, propagate ? "kM_L=kM_R propagated" : "per-motor preserved");
    }
    else if (args.argc == 1)
    {
        printf("kV = %.7f V/(mm/s) [= 1/Km]\n", settings_.kV);
    }
}

void DriverLab::cmdSetAccFF(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.0f, 0.01f, val))
    {
        const bool propagate = std::fabs(val - settings_.kA) > 1e-9f;
        settings_.kA         = val;
        if (propagate)
        {
            settings_.kA_L = val;
            settings_.kA_R = val;
        }
        printf("kA = %.7f V/(mm/s^2) (%s)\n", settings_.kA,
               propagate ? "kA_L=kA_R propagated" : "per-motor preserved");
    }
    else if (args.argc == 1)
    {
        printf("kA = %.7f V/(mm/s^2) [= Tm/Km]\n", settings_.kA);
    }
}

void DriverLab::cmdBattery()
{
    printf("Battery: %.2f V (ADC: %u)\n", battery_->voltage(), battery_->raw_adc());
}

void DriverLab::cmdEncoders()
{
    printf("Position:\n");
    if (left_encoder_)
    {
        const float left_mm = left_encoder_->ticks() * MM_PER_TICK;
        printf("  Left:  %ld ticks = %.1f mm\n", static_cast<long>(left_encoder_->ticks()),
               left_mm);
    }
    else
    {
        printf("  Left:  N/A (not connected)\n");
    }
    if (right_encoder_)
    {
        const float right_mm = right_encoder_->ticks() * MM_PER_TICK;
        printf("  Right: %ld ticks = %.1f mm\n", static_cast<long>(right_encoder_->ticks()),
               right_mm);
    }
    else
    {
        printf("  Right: N/A (not connected)\n");
    }

    printf("Velocity (cached, last tick):\n");
    printf("  Avg: %.1f mm/s\n", (left_velocity_mmps_ + right_velocity_mmps_) / 2.0f);
    printf("  L: %.1f mm/s  R: %.1f mm/s\n", left_velocity_mmps_, right_velocity_mmps_);
}

void DriverLab::cmdYaw()
{
    printf("Yaw: %.2f deg\n", imu_->robot_angle());
}

void DriverLab::cmdYawVel()
{
    printf("Angular velocity: %.2f deg/s\n", imu_->robot_omega());
}

void DriverLab::cmdYawContinuous(const DriverLabArgs& args)
{
    uint32_t duration_ms = 10000;
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

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t       elapsed    = 0;
    while (elapsed < duration_ms)
    {
        elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
        // IMU is ISR-driven; just read.
        printf("%7lu  %8.2f  %8.2f\n", static_cast<unsigned long>(elapsed), imu_->robot_angle(),
               imu_->robot_omega());
        sleep_ms(interval_ms);
    }
    printf("=== Done ===\n");
}

void DriverLab::cmdYawReset()
{
    imu_->reset();
    printf("Yaw and angular velocity reset to 0\n");
}

void DriverLab::cmdLeftTof()
{
    if (left_tof_ != nullptr)
        printf("Left ToF: %.0f mm\n", left_tof_->get_distance());
    else
        printf("Left ToF not available\n");
}

void DriverLab::cmdFrontTof()
{
    if (front_tof_ != nullptr)
        printf("Front ToF: %.0f mm\n", front_tof_->get_distance());
    else
        printf("Front ToF not available\n");
}

void DriverLab::cmdRightTof()
{
    if (right_tof_ != nullptr)
        printf("Right ToF: %.0f mm\n", right_tof_->get_distance());
    else
        printf("Right ToF not available\n");
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

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t       elapsed    = 0;
    while (elapsed < duration_ms)
    {
        elapsed                = to_ms_since_boot(get_absolute_time()) - start_time;
        const float left_dist  = (left_tof_ != nullptr) ? left_tof_->get_distance() : 0.0f;
        const float front_dist = (front_tof_ != nullptr) ? front_tof_->get_distance() : 0.0f;
        const float right_dist = (right_tof_ != nullptr) ? right_tof_->get_distance() : 0.0f;
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
    const float left_mm = left_encoder_->ticks() * MM_PER_TICK;
    printf("Left: %.2f mm (%ld ticks)\n", left_mm, static_cast<long>(left_encoder_->ticks()));
}

void DriverLab::cmdRightEncoder()
{
    if (!right_encoder_)
    {
        printf("Right encoder not connected\n");
        return;
    }
    const float right_mm = right_encoder_->ticks() * MM_PER_TICK;
    printf("Right: %.2f mm (%ld ticks)\n", right_mm, static_cast<long>(right_encoder_->ticks()));
}

void DriverLab::cmdEncoderReset()
{
    if (left_encoder_)
        left_encoder_->reset();
    if (right_encoder_)
        right_encoder_->reset();
    prev_left_ticks_   = 0;
    prev_right_ticks_  = 0;
    left_position_mm_  = 0.0f;
    right_position_mm_ = 0.0f;
    resetEncoderMA();
    left_velocity_mmps_  = 0.0f;
    right_velocity_mmps_ = 0.0f;
    printf("Encoders reset to 0\n");
}

void DriverLab::cmdEncoderContinuous(const DriverLabArgs& args)
{
    uint32_t duration_ms = 10000;
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

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t       elapsed    = 0;
    while (elapsed < duration_ms)
    {
        elapsed              = to_ms_since_boot(get_absolute_time()) - start_time;
        const float left_mm  = left_encoder_ ? left_encoder_->ticks() * MM_PER_TICK : 0.0f;
        const float right_mm = right_encoder_ ? right_encoder_->ticks() * MM_PER_TICK : 0.0f;
        printf("%7lu  %8.2f  %8.2f\n", static_cast<unsigned long>(elapsed), left_mm, right_mm);
        sleep_ms(interval_ms);
    }
    printf("=== Done ===\n");
}

void DriverLab::cmdOpenLoop(const DriverLabArgs& args)
{
    float    max_v     = 6.0f;
    float    step_v    = 1.0f;
    uint32_t settle_ms = 2000;
    if (args.argc > 1)
        max_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        step_v = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        settle_ms = static_cast<uint32_t>(atoi(args.argv[3]));
    startOpenLoopTrial(max_v, step_v, settle_ms);
}

void DriverLab::cmdStep(const DriverLabArgs& args)
{
    float    step_v      = 3.0f;
    uint32_t duration_ms = 1000;
    if (args.argc > 1)
        step_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[2]));
    startStepTrial(step_v, duration_ms);
}

void DriverLab::cmdMove(const DriverLabArgs& args)
{
    float dist  = 480.0f;
    float speed = 200.0f;
    float accel = 500.0f;
    int   mode  = 0; // FULL_CONTROL (FF+PD) — motorlab default
    if (args.argc > 1)
        dist = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        speed = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        accel = static_cast<float>(atof(args.argv[3]));
    if (args.argc > 4)
        mode = atoi(args.argv[4]);
    startMoveTrial(dist, speed, accel, mode);
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
    startTurnTrial(degrees, omega, alpha);
}

void DriverLab::cmdTurnOpenLoop(const DriverLabArgs& args)
{
    float    max_v     = 3.0f;
    float    step_v    = 0.5f;
    uint32_t settle_ms = 800;
    if (args.argc > 1)
        max_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        step_v = static_cast<float>(atof(args.argv[2]));
    if (args.argc > 3)
        settle_ms = static_cast<uint32_t>(atoi(args.argv[3]));
    startTurnOpenLoopTrial(max_v, step_v, settle_ms);
}

void DriverLab::cmdTurnStep(const DriverLabArgs& args)
{
    float    diff_v      = 1.5f;
    uint32_t duration_ms = 1000;
    if (args.argc > 1)
        diff_v = static_cast<float>(atof(args.argv[1]));
    if (args.argc > 2)
        duration_ms = static_cast<uint32_t>(atoi(args.argv[2]));
    startTurnStepTrial(diff_v, duration_ms);
}

void DriverLab::cmdSetRotZeta(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.1f, 2.0f, val))
    {
        settings_.rot_zeta = val;
        settings_.recalculateRotation();
        printf("rot_zeta = %.5f (turnKP, turnKD updated)\n", settings_.rot_zeta);
    }
    else if (args.argc == 1)
    {
        printf("rot_zeta = %.5f\n", settings_.rot_zeta);
    }
}

void DriverLab::cmdSetRotTd(const DriverLabArgs& args)
{
    float val;
    if (parseFloat(args, 1, 0.001f, 1.0f, val))
    {
        settings_.rot_td = val;
        settings_.recalculateRotation();
        printf("rot_td = %.5f (turnKP, turnKD updated)\n", settings_.rot_td);
    }
    else if (args.argc == 1)
    {
        printf("rot_td = %.5f\n", settings_.rot_td);
    }
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
    trial_ = TrialState::Idle; // abort any active trial
    printf("Motors stopped (any trial aborted)\n");
    printBluetoothDiagnostics();
}

void DriverLab::cmdBtLog(const DriverLabArgs& args)
{
    if (args.argc > 1)
    {
        if (strcmp(args.argv[1], "RESET") == 0)
        {
            BluetoothStdio::resetDiagnostics();
            printf("Bluetooth TX diagnostics reset\n");
        }
        else
        {
            char*      end    = nullptr;
            const long ms_val = strtol(args.argv[1], &end, 10);
            if (end == args.argv[1] || *end != '\0' || ms_val < 0 || ms_val > 1000)
            {
                printf("Usage: BTLOG [0..1000|RESET]  (0 = full-rate Bluetooth CSV)\n");
                return;
            }

            BluetoothStdio::setCsvIntervalMs(static_cast<uint32_t>(ms_val));
            printf("Bluetooth CSV interval set to %lu ms", static_cast<unsigned long>(ms_val));
            if (ms_val == 0)
                printf(" (full-rate)");
            printf("\n");
        }
    }

    const BluetoothStdio::Diagnostics bt = BluetoothStdio::diagnostics();
    printf("BT TX: csv_interval=%lu ms, depth=%lu/%lu, max_depth=%lu, suppressed_rows=%lu, "
           "dropped_bytes=%lu\n",
           static_cast<unsigned long>(bt.csv_interval_ms), static_cast<unsigned long>(bt.depth),
           static_cast<unsigned long>(bt.ring_size), static_cast<unsigned long>(bt.max_depth),
           static_cast<unsigned long>(bt.suppressed_csv_lines),
           static_cast<unsigned long>(bt.dropped_bytes));
}

void DriverLab::cmdExport()
{
    const float battery_volts = batteryVoltage();
    // Refuse to bake calibration constants from an implausible reading.
    // 5.0 V is well below any healthy 2S LiPo (~6 V cutoff) — a USB-only or
    // mid-hot-plug filter state will fail closed instead of silently producing
    // wrong duty constants that persist into every future maze run.
    if (battery_volts < 5.0f)
    {
        printf("EXPORT: battery reads %.2f V — below 2S floor (5.0 V). "
               "Pack disconnected or filter not yet settled. Refusing.\n",
               battery_volts);
        return;
    }

    const float kvl_duty =
        (std::fabs(settings_.kM_L) > 1e-6f) ? (1.0f / (settings_.kM_L * battery_volts)) : 0.0f;
    const float kvr_duty =
        (std::fabs(settings_.kM_R) > 1e-6f) ? (1.0f / (settings_.kM_R * battery_volts)) : 0.0f;
    const float ksl_duty = settings_.kS_L / battery_volts;
    const float ksr_duty = settings_.kS_R / battery_volts;
    const float kal_duty = settings_.kA_L / battery_volts;
    const float kar_duty = settings_.kA_R / battery_volts;

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
    printf("// Acceleration feedforward (per-motor, derived from Tm / kM_L|R)\n");
    printf("#define FORWARD_KAL %.7ff  // Left accel gain (duty per mm/s^2)\n", kal_duty);
    printf("#define FORWARD_KAR %.7ff  // Right accel gain (duty per mm/s^2)\n", kar_duty);
    printf("\n");
    printf("// Raw DriverLab values (for reference):\n");
    printf("//   kM_combined = %.2f mm/s/V\n", settings_.kM);
    printf("//   kM_L = %.2f mm/s/V, kM_R = %.2f mm/s/V\n", settings_.kM_L, settings_.kM_R);
    printf("//   kS_L = %.3f V, kS_R = %.3f V\n", settings_.kS_L, settings_.kS_R);
    printf("//   kA_L = %.7f V/(mm/s^2), kA_R = %.7f V/(mm/s^2)\n", settings_.kA_L, settings_.kA_R);
    printf("//   Tm = %.5f s\n", settings_.tm);
    printf("\n");
    printf("// Rotation plant model (from TURN-OL + TURN-STEP)\n");
    printf("#define ROT_KM %.2ff   // deg/s per volt of differential drive\n", settings_.rot_kM);
    printf("#define ROT_TM %.5ff // seconds -- rotational time constant\n", settings_.rot_tm);
    printf("// ROT_KP / ROT_KD are now formula-derived in tuning.h.\n");
    printf("// Resulting gains: turnKP = %.4f, turnKD = %.4f\n", settings_.turnKP,
           settings_.turnKD);
    printf("// ============================================================\n");
    printf("\n");
}

void DriverLab::cmdGpioDiag(const DriverLabArgs& args)
{
    (void)args;
    printf("\n=== Raw GPIO Diagnostic ===\n");
    printf("Reading GP%d (channel A) and GP%d (channel B)\n", PIN_ENCODER_R_A, PIN_ENCODER_R_B);
    printf("Slowly spin right wheel and watch for changes\n");
    printf("Both pins should toggle. If only one changes -> wiring issue\n\n");

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
        const bool pin_a = gpio_get(PIN_ENCODER_R_A);
        const bool pin_b = gpio_get(PIN_ENCODER_R_B);
        const int  state = (pin_a ? 2 : 0) | (pin_b ? 1 : 0);
        printf("%6d     %d       %d      %d\n", i, pin_a, pin_b, state);
        sleep_ms(100);
    }

    printf("\n=== Analysis ===\n");
    printf("Expected: Both columns toggle between 0 and 1 as wheel spins\n");
    printf("Expected: State cycles through 0->1->3->2->0 (or reverse)\n");
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
    const float position = line_sensor_->get_position();
    const bool  on_line  = line_sensor_->on_line();
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
    const bool intersection = line_sensor_->detect_intersection();
    printf("Intersection: %s\n", intersection ? "YES" : "NO");
}

void DriverLab::cmdLineContinuous(const DriverLabArgs& args)
{
    if (line_sensor_ == nullptr)
    {
        printf("Line sensor not available\n");
        return;
    }

    uint32_t duration_ms = 10000;
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

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t       elapsed    = 0;
    while (elapsed < duration_ms)
    {
        elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
        line_sensor_->read();
        const float position     = line_sensor_->get_position();
        const bool  intersection = line_sensor_->detect_intersection();
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
    if (history_count_ > 0)
    {
        const int last_idx =
            (history_write_idx_ - 1 + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
        if (strcmp(input_buffer_, history_[last_idx]) == 0)
            return;
    }

    strncpy(history_[history_write_idx_], input_buffer_, DRIVERLAB_INPUT_BUFFER_SIZE - 1);
    history_[history_write_idx_][DRIVERLAB_INPUT_BUFFER_SIZE - 1] = '\0';

    history_write_idx_ = (history_write_idx_ + 1) % DRIVERLAB_HISTORY_SIZE;
    if (history_count_ < DRIVERLAB_HISTORY_SIZE)
        history_count_++;
}

void DriverLab::cmdRepeat()
{
    if (history_count_ == 0)
    {
        printf("No command history\n");
        return;
    }
    const int last_idx = (history_write_idx_ - 1 + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
    strncpy(input_buffer_, history_[last_idx], DRIVERLAB_INPUT_BUFFER_SIZE);
    printf("Repeating: %s\n", input_buffer_);

    DriverLabArgs args = tokenize();
    if (args.argc > 0)
        executeCommand(args);
}

void DriverLab::cmdHistory()
{
    if (history_count_ == 0)
    {
        printf("No command history\n");
        return;
    }
    printf("Command History:\n");
    const int oldest =
        (history_write_idx_ - history_count_ + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
    for (int i = 0; i < history_count_; i++)
    {
        const int idx = (oldest + i) % DRIVERLAB_HISTORY_SIZE;
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
    const int oldest =
        (history_write_idx_ - history_count_ + DRIVERLAB_HISTORY_SIZE) % DRIVERLAB_HISTORY_SIZE;
    const int idx = (oldest + selection - 1) % DRIVERLAB_HISTORY_SIZE;
    strncpy(input_buffer_, history_[idx], DRIVERLAB_INPUT_BUFFER_SIZE);
    printf("Executing: %s\n", input_buffer_);

    DriverLabArgs args = tokenize();
    if (args.argc > 0)
        executeCommand(args);
}

// ============================================================================
// Motor Direction Pin Diagnostic
// ============================================================================

void DriverLab::cmdDirTest(const DriverLabArgs& args)
{
    (void)args;
    printf("\n=== Motor Direction Pin Test ===\n");
    printf("This applies constant 30%% PWM and toggles DIR pins\n");
    printf("Watch motors - they should alternate direction every 2 seconds\n");
    printf("Press any key to stop\n\n");

    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
        ;
    while (uart_is_readable(uart0))
        uart_getc(uart0);

    const uint     left_slice    = pwm_gpio_to_slice_num(PIN_MOTOR_L_PWM);
    const uint     left_channel  = pwm_gpio_to_channel(PIN_MOTOR_L_PWM);
    const uint     right_slice   = pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM);
    const uint     right_channel = pwm_gpio_to_channel(PIN_MOTOR_R_PWM);
    const uint16_t pwm_level     = static_cast<uint16_t>(PWM_WRAP * 0.3f);

    int cycle = 0;
    while (true)
    {
        const int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT)
        {
            printf("\nStopped by user\n");
            break;
        }

        const bool dir_state = (cycle % 2 == 0);
        printf("Cycle %d: DIR_L=%d, DIR_R=%d\n", cycle + 1, dir_state, dir_state);
        cycle++;

        gpio_put(PIN_MOTOR_L_DIR, dir_state);
        gpio_put(PIN_MOTOR_R_DIR, dir_state);
        pwm_set_chan_level(left_slice, left_channel, pwm_level);
        pwm_set_chan_level(right_slice, right_channel, pwm_level);

        sleep_ms(2000);
    }

    stopMotors();
    printf("=== Test Complete ===\n");
}
