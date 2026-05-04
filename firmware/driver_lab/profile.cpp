#include "driver_lab/profile.h"

#include <cmath>

DriverLabProfile::DriverLabProfile()
    : state_(DriverLabProfileState::IDLE), target_distance_(0.0f), target_speed_(0.0f),
      top_speed_(0.0f), final_speed_(0.0f), acceleration_(0.0f), current_acceleration_(0.0f),
      position_(0.0f), speed_(0.0f), direction_(1.0f)
{
}

void DriverLabProfile::start(float distance, float top_speed, float acceleration, float final_speed)
{
    direction_       = (distance >= 0.0f) ? 1.0f : -1.0f;
    target_distance_ = std::fabs(distance);
    top_speed_       = std::fabs(top_speed);
    final_speed_     = std::fabs(final_speed);
    acceleration_    = std::fabs(acceleration);
    target_speed_    = top_speed_;

    position_             = 0.0f;
    speed_                = 0.0f;
    current_acceleration_ = 0.0f;

    if (target_distance_ <= 0.0f || acceleration_ <= 0.0f)
    {
        state_ = DriverLabProfileState::FINISHED;
        return;
    }

    state_ = DriverLabProfileState::ACCELERATING;
}

void DriverLabProfile::update(float dt)
{
    if (dt <= 0.0f || state_ == DriverLabProfileState::IDLE ||
        state_ == DriverLabProfileState::FINISHED)
    {
        return;
    }

    const float remaining = target_distance_ - position_;
    if (state_ == DriverLabProfileState::ACCELERATING && remaining <= brakingDistance())
    {
        state_ = DriverLabProfileState::BRAKING;
    }

    if (state_ == DriverLabProfileState::BRAKING)
    {
        target_speed_ = (final_speed_ == 0.0f) ? 5.0f : final_speed_;
    }
    else
    {
        target_speed_ = top_speed_;
    }

    const float old_speed = speed_;
    const float delta_v   = acceleration_ * dt;
    if (speed_ < target_speed_)
    {
        speed_ += delta_v;
        if (speed_ > target_speed_)
            speed_ = target_speed_;
    }
    else if (speed_ > target_speed_)
    {
        speed_ -= delta_v;
        if (speed_ < target_speed_)
            speed_ = target_speed_;
    }

    current_acceleration_ = direction_ * (speed_ - old_speed) / dt;
    position_ += speed_ * dt;

    if (target_distance_ - position_ < 0.125f)
    {
        position_             = target_distance_;
        speed_                = final_speed_;
        current_acceleration_ = 0.0f;
        state_                = DriverLabProfileState::FINISHED;
    }
}

float DriverLabProfile::brakingDistance() const
{
    float delta_v_squared = speed_ * speed_ - final_speed_ * final_speed_;
    if (delta_v_squared <= 0.0f || acceleration_ <= 0.0f)
    {
        return 0.0f;
    }
    return delta_v_squared / (2.0f * acceleration_);
}

float DriverLabProfile::acceleration() const
{
    return current_acceleration_;
}

void DriverLabProfile::reset()
{
    state_                = DriverLabProfileState::IDLE;
    target_distance_      = 0.0f;
    target_speed_         = 0.0f;
    top_speed_            = 0.0f;
    final_speed_          = 0.0f;
    acceleration_         = 0.0f;
    current_acceleration_ = 0.0f;
    position_             = 0.0f;
    speed_                = 0.0f;
    direction_            = 1.0f;
}
