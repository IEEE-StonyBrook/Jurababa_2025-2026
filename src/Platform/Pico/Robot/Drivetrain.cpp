#include "Platform/Pico/Robot/Drivetrain.h"
#include "Platform/Pico/Config.h"

Drivetrain::Drivetrain(Motor* left_motor, Motor* right_motor,
                       Encoder* left_encoder, Encoder* right_encoder,
                       BatteryMonitor* battery_monitor)
    : left_motor_(left_motor),
      right_motor_(right_motor),
      left_encoder_(left_encoder),
      right_encoder_(right_encoder),
      battery_monitor_(battery_monitor)
{
}

void Drivetrain::reset()
{
    left_encoder_->reset();
    right_encoder_->reset();

    previous_left_ticks_ = left_encoder_->getTickCount();
    previous_right_ticks_ = right_encoder_->getTickCount();

    left_velocity_mm_per_second_ = 0.0f;
    right_velocity_mm_per_second_ = 0.0f;

    // Reset delta tracking for incremental error accumulation
    last_left_position_mm_ = 0.0f;
    last_right_position_mm_ = 0.0f;
}

// ============================================================
// Type-Safe API (Preferred)
// ============================================================

float Drivetrain::getMotorDistanceMM(WheelSide side)
{
    bool is_left = (side == WheelSide::LEFT);
    int tick_count = is_left ? left_encoder_->getTickCount() : right_encoder_->getTickCount();
    return tick_count * MM_PER_TICK;
}

float Drivetrain::getMotorVelocityMMps(WheelSide side)
{
    return (side == WheelSide::LEFT) ? left_velocity_mm_per_second_
                                      : right_velocity_mm_per_second_;
}

float Drivetrain::getFeedforward(WheelSide side, float wheel_speed_mm_per_second,
                                float wheel_accel_mm_per_second2)
{
    if (std::fabs(wheel_speed_mm_per_second) < DRIVETRAIN_FF_DEADZONE_MMPS)
        return 0.0f;

    bool is_left = (side == WheelSide::LEFT);

    // TODO: Mazerunner-core has per-motor Ka tuning for better asymmetry compensation
    if (wheel_speed_mm_per_second > 0.0f)
    {
        float kv = is_left ? FORWARD_KVL : FORWARD_KVR;
        float ks = is_left ? FORWARD_KSL : FORWARD_KSR;
        float ka = is_left ? FORWARD_KAL : FORWARD_KAR;
        return kv * wheel_speed_mm_per_second + ks + ka * wheel_accel_mm_per_second2;
    }
    else
    {
        float kv = is_left ? REVERSE_KVL : REVERSE_KVR;
        float ks = is_left ? REVERSE_KSL : REVERSE_KSR;
        float ka = is_left ? REVERSE_KAL : REVERSE_KAR;
        return kv * wheel_speed_mm_per_second - ks + ka * wheel_accel_mm_per_second2;
    }
}

float Drivetrain::getMotorDeltaMM(WheelSide side)
{
    // Get current position
    float current_pos = getMotorDistanceMM(side);

    // Get last position reference
    float& last_pos = (side == WheelSide::LEFT) ? last_left_position_mm_
                                                  : last_right_position_mm_;

    // Calculate delta
    float delta = current_pos - last_pos;

    // Update last position for next call
    last_pos = current_pos;

    return delta;
}

// ============================================================
// Legacy String API (Deprecated - for backward compatibility)
// ============================================================

float Drivetrain::getMotorDistanceMM(std::string side)
{
    if (side == "left")
        return getMotorDistanceMM(WheelSide::LEFT);
    if (side == "right")
        return getMotorDistanceMM(WheelSide::RIGHT);

    LOG_ERROR("Drivetrain::getMotorDistanceMM - Invalid side string: " + side);
    return 0.0f;
}

float Drivetrain::getMotorVelocityMMps(std::string side)
{
    if (side == "left")
        return getMotorVelocityMMps(WheelSide::LEFT);
    if (side == "right")
        return getMotorVelocityMMps(WheelSide::RIGHT);

    LOG_ERROR("Drivetrain::getMotorVelocityMMps - Invalid side string: " + side);
    return 0.0f;
}

float Drivetrain::getFeedforward(std::string side, float wheel_speed_mm_per_second,
                                float wheel_accel_mm_per_second2)
{
    if (side == "left")
        return getFeedforward(WheelSide::LEFT, wheel_speed_mm_per_second, wheel_accel_mm_per_second2);
    if (side == "right")
        return getFeedforward(WheelSide::RIGHT, wheel_speed_mm_per_second, wheel_accel_mm_per_second2);

    LOG_ERROR("Drivetrain::getFeedforward - Invalid side string: " + side);
    return 0.0f;
}

// ============================================================
// Common Operations
// ============================================================

void Drivetrain::updateVelocities(float time_delta)
{
    if (time_delta < DRIVETRAIN_MIN_DT) return;

    int32_t current_left_ticks = left_encoder_->getTickCount();
    int32_t current_right_ticks = right_encoder_->getTickCount();

    int32_t delta_left = current_left_ticks - previous_left_ticks_;
    int32_t delta_right = current_right_ticks - previous_right_ticks_;

    previous_left_ticks_ = current_left_ticks;
    previous_right_ticks_ = current_right_ticks;

    left_velocity_mm_per_second_ = (delta_left * MM_PER_TICK) / time_delta;
    right_velocity_mm_per_second_ = (delta_right * MM_PER_TICK) / time_delta;

    if (left_velocity_mm_per_second_ > DRIVETRAIN_MAX_VELOCITY_MMPS ||
        right_velocity_mm_per_second_ > DRIVETRAIN_MAX_VELOCITY_MMPS)
    {
        LOG_ERROR("Velocity spike: L=" + std::to_string(left_velocity_mm_per_second_) +
                  " R=" + std::to_string(right_velocity_mm_per_second_) + " mm/s");
    }
}

void Drivetrain::setDuty(float left_duty, float right_duty)
{
    left_motor_->applyDuty(left_duty);
    right_motor_->applyDuty(right_duty);
}

void Drivetrain::setVoltage(float left_volts, float right_volts)
{
    float battery_volts = getBatteryVoltage();
    left_motor_->applyVoltage(left_volts, battery_volts);
    right_motor_->applyVoltage(right_volts, battery_volts);
}

float Drivetrain::getBatteryVoltage() const
{
    if (battery_monitor_ != nullptr)
    {
        return battery_monitor_->getVoltage();
    }
    return DEFAULT_BATTERY_VOLTAGE;
}

void Drivetrain::stop()
{
    left_motor_->stopMotor();
    right_motor_->stopMotor();
}