#include "motor_lab/profile.h"

#include <cmath>

MotorLabProfile::MotorLabProfile()
    : state_(MotorLabProfileState::IDLE), target_distance_(0.0f), top_speed_(0.0f),
      final_speed_(0.0f), acceleration_(0.0f), position_(0.0f), speed_(0.0f), direction_(1.0f)
{
}

void MotorLabProfile::start(float distance, float top_speed, float acceleration, float final_speed)
{
    // Determine direction
    direction_       = (distance >= 0.0f) ? 1.0f : -1.0f;
    target_distance_ = std::fabs(distance);
    top_speed_       = std::fabs(top_speed);
    final_speed_     = std::fabs(final_speed);
    acceleration_    = std::fabs(acceleration);

    // Reset runtime state
    position_ = 0.0f;
    speed_    = 0.0f;

    // Start accelerating
    state_ = MotorLabProfileState::ACCELERATING;
}

void MotorLabProfile::update(float dt)
{
    if (dt <= 0.0f || state_ == MotorLabProfileState::IDLE ||
        state_ == MotorLabProfileState::FINISHED)
    {
        return;
    }

    // Calculate remaining distance
    float remaining = target_distance_ - position_;

    // Margin to ensure we actually stop (from UKMARS)
    const float STOP_MARGIN       = 5.0f;
    float       braking_threshold = brakingDistance() + STOP_MARGIN;

    switch (state_)
    {
        case MotorLabProfileState::ACCELERATING:
            speed_ += acceleration_ * dt;
            if (speed_ >= top_speed_)
            {
                speed_ = top_speed_;
                state_ = MotorLabProfileState::CRUISING;
            }
            if (remaining <= braking_threshold)
            {
                state_ = MotorLabProfileState::BRAKING;
            }
            break;

        case MotorLabProfileState::CRUISING:
            speed_ = top_speed_;
            if (remaining <= braking_threshold)
            {
                state_ = MotorLabProfileState::BRAKING;
            }
            break;

        case MotorLabProfileState::BRAKING:
            speed_ -= acceleration_ * dt;
            if (speed_ <= final_speed_)
            {
                speed_ = final_speed_;
            }
            if (remaining <= 0.0f || (final_speed_ == 0.0f && speed_ <= 0.0f))
            {
                speed_    = final_speed_;
                position_ = target_distance_;
                state_    = MotorLabProfileState::FINISHED;
            }
            break;

        case MotorLabProfileState::IDLE:
        case MotorLabProfileState::FINISHED:
            // Already handled at function entry
            break;
    }

    // Update position based on current speed
    position_ += speed_ * dt;

    // Clamp position to target
    if (position_ >= target_distance_)
    {
        position_ = target_distance_;
        if (state_ != MotorLabProfileState::FINISHED)
        {
            state_ = MotorLabProfileState::FINISHED;
            speed_ = final_speed_;
        }
    }
}

float MotorLabProfile::brakingDistance() const
{
    // Distance = (v^2 - v_final^2) / (2 * a)
    // This is the standard kinematic equation for deceleration
    float delta_v_squared = speed_ * speed_ - final_speed_ * final_speed_;
    if (delta_v_squared <= 0.0f || acceleration_ <= 0.0f)
    {
        return 0.0f;
    }
    return delta_v_squared / (2.0f * acceleration_);
}

float MotorLabProfile::acceleration() const
{
    switch (state_)
    {
        case MotorLabProfileState::ACCELERATING:
            return direction_ * acceleration_;
        case MotorLabProfileState::BRAKING:
            return -direction_ * acceleration_;
        default:
            return 0.0f;
    }
}

void MotorLabProfile::reset()
{
    state_           = MotorLabProfileState::IDLE;
    position_        = 0.0f;
    speed_           = 0.0f;
    target_distance_ = 0.0f;
}
