#include "control/drivetrain.h"

#include <cmath>
#include <string>

#include "config/config.h"
#include "config/motion.h"

Drivetrain::Drivetrain(Motor* left_motor, Motor* right_motor, Encoder* left_encoder,
                       Encoder* right_encoder, Battery* battery)
    : left_motor_(left_motor), right_motor_(right_motor), left_encoder_(left_encoder),
      right_encoder_(right_encoder), battery_(battery),
      ff_fwd_left_{FORWARD_KVL, FORWARD_KSL, FORWARD_KAL},
      ff_fwd_right_{FORWARD_KVR, FORWARD_KSR, FORWARD_KAR}
{
}

void Drivetrain::reset()
{
    if (left_encoder_)
    {
        left_encoder_->reset();
        prev_left_ticks_ = left_encoder_->ticks();
    }
    else
    {
        prev_left_ticks_ = 0;
    }

    if (right_encoder_)
    {
        right_encoder_->reset();
        prev_right_ticks_ = right_encoder_->ticks();
    }
    else
    {
        prev_right_ticks_ = 0;
    }

    left_velocity_mmps_  = 0.0f;
    right_velocity_mmps_ = 0.0f;

    fwd_change_mm_ = 0.0f;
}

float Drivetrain::position(WheelSide side)
{
    bool     is_left = (side == WheelSide::LEFT);
    Encoder* enc     = is_left ? left_encoder_ : right_encoder_;
    if (!enc)
        return 0.0f;
    return enc->ticks() * MM_PER_TICK;
}

float Drivetrain::velocity(WheelSide side)
{
    return (side == WheelSide::LEFT) ? left_velocity_mmps_ : right_velocity_mmps_;
}

float Drivetrain::feedforward(WheelSide side, float speed_mmps, float accel_mmps2)
{
    bool            is_left = (side == WheelSide::LEFT);
    const FFCoeffs& c       = is_left ? ff_fwd_left_ : ff_fwd_right_;

    // Mazerunner shape: KV*v + sign(v)*KS + KA*a, with no deadzone.
    // KS is zero-d only when v is exactly 0 (stationary hold). At any nonzero
    // command the static-friction term must be present, otherwise low-speed
    // commands produce zero motor voltage and the wheel stalls.
    float ks_term = 0.0f;
    if (speed_mmps > 0.0f)
        ks_term = c.ks;
    else if (speed_mmps < 0.0f)
        ks_term = -c.ks;
    return c.kv * speed_mmps + ks_term + c.ka * accel_mmps2;
}

void Drivetrain::setFeedforward(WheelSide side, float kv, float ks, float ka)
{
    if (side == WheelSide::LEFT)
        ff_fwd_left_ = {kv, ks, ka};
    else
        ff_fwd_right_ = {kv, ks, ka};
}

void Drivetrain::update()
{
    float left_change_mm  = 0.0f;
    float right_change_mm = 0.0f;

    if (left_encoder_)
    {
        int32_t curr_left = left_encoder_->ticks();
        int32_t d_left    = curr_left - prev_left_ticks_;
        prev_left_ticks_  = curr_left;
        left_change_mm    = d_left * MM_PER_TICK;
        // Velocity is diagnostic only — PD math uses the per-tick delta.
        // At 500 Hz the per-tick speed is quantized to ~185 mm/s steps for a
        // single tick of motion, so this number is noisy by design.
        left_velocity_mmps_ = left_change_mm / LOOP_INTERVAL_S;
    }

    if (right_encoder_)
    {
        int32_t curr_right   = right_encoder_->ticks();
        int32_t d_right      = curr_right - prev_right_ticks_;
        prev_right_ticks_    = curr_right;
        right_change_mm      = d_right * MM_PER_TICK;
        right_velocity_mmps_ = right_change_mm / LOOP_INTERVAL_S;
    }

    fwd_change_mm_ = 0.5f * (left_change_mm + right_change_mm);
}

void Drivetrain::setVoltage(float left_volts, float right_volts)
{
    float batt = batteryVoltage();
    left_motor_->set_motor_volts(left_volts, batt);
    right_motor_->set_motor_volts(right_volts, batt);
}

float Drivetrain::batteryVoltage() const
{
    if (battery_ != nullptr)
    {
        return battery_->voltage();
    }
    return DEFAULT_BATTERY_VOLTAGE;
}

void Drivetrain::stop()
{
    left_motor_->stop();
    right_motor_->stop();
}
