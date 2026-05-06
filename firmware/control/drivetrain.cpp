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

    for (int i = 0; i < ENCODER_AVERAGER_LENGTH; i++)
    {
        left_history_[i]  = 0;
        right_history_[i] = 0;
    }
    left_history_total_  = 0;
    right_history_total_ = 0;
    averager_index_      = 0;

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

int32_t Drivetrain::ticks(WheelSide side) const
{
    Encoder* enc = (side == WheelSide::LEFT) ? left_encoder_ : right_encoder_;
    return enc != nullptr ? enc->ticks() : 0;
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
    int d_left  = 0;
    int d_right = 0;

    if (left_encoder_)
    {
        int32_t curr_left = left_encoder_->ticks();
        d_left            = static_cast<int>(curr_left - prev_left_ticks_);
        prev_left_ticks_  = curr_left;
    }
    if (right_encoder_)
    {
        int32_t curr_right = right_encoder_->ticks();
        d_right            = static_cast<int>(curr_right - prev_right_ticks_);
        prev_right_ticks_  = curr_right;
    }

    // 8-tap moving averager — drop the oldest sample at this index, add the
    // newest. Mirrors ukmars/motorlab/src/encoders.h::update(). Both the PD's
    // measured_change input (fwdChangeMm()) and external velocity observers
    // see the smoothed value.
    left_history_total_ -= left_history_[averager_index_];
    right_history_total_ -= right_history_[averager_index_];
    left_history_total_ += d_left;
    right_history_total_ += d_right;
    left_history_[averager_index_]  = static_cast<int8_t>(d_left);
    right_history_[averager_index_] = static_cast<int8_t>(d_right);
    averager_index_                 = (averager_index_ + 1) % ENCODER_AVERAGER_LENGTH;

    constexpr float inv_avg      = 1.0f / static_cast<float>(ENCODER_AVERAGER_LENGTH);
    const float     left_change  = static_cast<float>(left_history_total_) * inv_avg * MM_PER_TICK;
    const float     right_change = static_cast<float>(right_history_total_) * inv_avg * MM_PER_TICK;

    left_velocity_mmps_  = left_change / LOOP_INTERVAL_S;
    right_velocity_mmps_ = right_change / LOOP_INTERVAL_S;
    fwd_change_mm_       = 0.5f * (left_change + right_change);
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
