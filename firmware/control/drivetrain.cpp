#include "control/drivetrain.h"

#include <cmath>
#include <string>

#include "config/config.h"

Drivetrain::Drivetrain(Motor* left_motor, Motor* right_motor,
                       Encoder* left_encoder, Encoder* right_encoder,
                       Battery* battery)
    : left_motor_(left_motor),
      right_motor_(right_motor),
      left_encoder_(left_encoder),
      right_encoder_(right_encoder),
      battery_(battery)
{
}

void Drivetrain::reset()
{
    left_encoder_->reset();
    right_encoder_->reset();

    prev_left_ticks_ = left_encoder_->ticks();
    prev_right_ticks_ = right_encoder_->ticks();

    left_velocity_mmps_ = 0.0f;
    right_velocity_mmps_ = 0.0f;

    last_left_pos_mm_ = 0.0f;
    last_right_pos_mm_ = 0.0f;
}

float Drivetrain::position(WheelSide side)
{
    bool is_left = (side == WheelSide::LEFT);
    int tick_count = is_left ? left_encoder_->ticks() : right_encoder_->ticks();
    return tick_count * MM_PER_TICK;
}

float Drivetrain::velocity(WheelSide side)
{
    return (side == WheelSide::LEFT) ? left_velocity_mmps_ : right_velocity_mmps_;
}

float Drivetrain::delta(WheelSide side)
{
    float current_pos = position(side);
    float& last_pos = (side == WheelSide::LEFT) ? last_left_pos_mm_ : last_right_pos_mm_;
    float d = current_pos - last_pos;
    last_pos = current_pos;
    return d;
}

float Drivetrain::feedforward(WheelSide side, float speed_mmps, float accel_mmps2)
{
    if (std::fabs(speed_mmps) < DRIVETRAIN_FF_DEADZONE_MMPS)
        return 0.0f;

    bool is_left = (side == WheelSide::LEFT);

    if (speed_mmps > 0.0f)
    {
        float kv = is_left ? FORWARD_KVL : FORWARD_KVR;
        float ks = is_left ? FORWARD_KSL : FORWARD_KSR;
        float ka = is_left ? FORWARD_KAL : FORWARD_KAR;
        return kv * speed_mmps + ks + ka * accel_mmps2;
    }
    else
    {
        float kv = is_left ? REVERSE_KVL : REVERSE_KVR;
        float ks = is_left ? REVERSE_KSL : REVERSE_KSR;
        float ka = is_left ? REVERSE_KAL : REVERSE_KAR;
        return kv * speed_mmps - ks + ka * accel_mmps2;
    }
}

void Drivetrain::update(float dt)
{
    if (dt < DRIVETRAIN_MIN_DT) return;

    int32_t curr_left = left_encoder_->ticks();
    int32_t curr_right = right_encoder_->ticks();

    int32_t d_left = curr_left - prev_left_ticks_;
    int32_t d_right = curr_right - prev_right_ticks_;

    prev_left_ticks_ = curr_left;
    prev_right_ticks_ = curr_right;

    left_velocity_mmps_ = (d_left * MM_PER_TICK) / dt;
    right_velocity_mmps_ = (d_right * MM_PER_TICK) / dt;

    if (left_velocity_mmps_ > DRIVETRAIN_MAX_VELOCITY_MMPS ||
        right_velocity_mmps_ > DRIVETRAIN_MAX_VELOCITY_MMPS)
    {
        LOG_ERROR("Velocity spike: L=" + std::to_string(left_velocity_mmps_) +
                  " R=" + std::to_string(right_velocity_mmps_) + " mm/s");
    }
}

void Drivetrain::setDuty(float left, float right)
{
    left_motor_->applyDuty(left);
    right_motor_->applyDuty(right);
}

void Drivetrain::setVoltage(float left_volts, float right_volts)
{
    float batt = batteryVoltage();
    left_motor_->applyVoltage(left_volts, batt);
    right_motor_->applyVoltage(right_volts, batt);
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
