#include "driver_lab/profile.h"

#include <cmath>

DriverLabProfile::DriverLabProfile()
    : state_(DriverLabProfileState::IDLE), target_distance_(0.0f), top_speed_(0.0f),
      final_speed_(0.0f), acceleration_(0.0f), position_(0.0f), speed_(0.0f), direction_(1.0f)
{
}

void DriverLabProfile::start(float distance, float top_speed, float acceleration, float final_speed)
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
    state_ = DriverLabProfileState::ACCELERATING;
}

void DriverLabProfile::update(float dt)
{
    if (dt <= 0.0f || state_ == DriverLabProfileState::IDLE ||
        state_ == DriverLabProfileState::FINISHED)
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
        case DriverLabProfileState::ACCELERATING:
            speed_ += acceleration_ * dt;
            if (speed_ >= top_speed_)
            {
                speed_ = top_speed_;
                state_ = DriverLabProfileState::CRUISING;
            }
            if (remaining <= braking_threshold)
            {
                state_ = DriverLabProfileState::BRAKING;
            }
            break;

        case DriverLabProfileState::CRUISING:
            speed_ = top_speed_;
            if (remaining <= braking_threshold)
            {
                state_ = DriverLabProfileState::BRAKING;
            }
            break;

        case DriverLabProfileState::BRAKING:
            speed_ -= acceleration_ * dt;
            if (speed_ <= final_speed_)
            {
                speed_ = final_speed_;
            }
            if (remaining <= 0.0f || (final_speed_ == 0.0f && speed_ <= 0.0f))
            {
                speed_    = final_speed_;
                position_ = target_distance_;
                state_    = DriverLabProfileState::FINISHED;
            }
            break;

        case DriverLabProfileState::IDLE:
        case DriverLabProfileState::FINISHED:
            // Already handled at function entry
            break;
    }

    // Update position based on current speed
    position_ += speed_ * dt;

    // Clamp position to target
    if (position_ >= target_distance_)
    {
        position_ = target_distance_;
        if (state_ != DriverLabProfileState::FINISHED)
        {
            state_ = DriverLabProfileState::FINISHED;
            speed_ = final_speed_;
        }
    }
}

float DriverLabProfile::brakingDistance() const
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

float DriverLabProfile::acceleration() const
{
    switch (state_)
    {
        case DriverLabProfileState::ACCELERATING:
            return direction_ * acceleration_;
        case DriverLabProfileState::BRAKING:
            return -direction_ * acceleration_;
        default:
            return 0.0f;
    }
}

void DriverLabProfile::reset()
{
    state_           = DriverLabProfileState::IDLE;
    position_        = 0.0f;
    speed_           = 0.0f;
    target_distance_ = 0.0f;
}
