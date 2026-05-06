#include "control/profile.h"

#include <cmath>

#include "config/motion.h"

Profile::Profile()
    : state_(State::Idle), direction_(1), target_distance_(0.0f), target_speed_(0.0f),
      top_speed_(0.0f), final_speed_(0.0f), acceleration_(0.0f), current_velocity_(0.0f),
      current_acceleration_(0.0f), current_position_(0.0f)
{
}

void Profile::start(float target_distance, float top_speed, float final_speed, float acceleration)
{
    start(target_distance, 0.0f, top_speed, final_speed, acceleration);
}

void Profile::start(float target_distance, float start_speed, float top_speed, float final_speed,
                    float acceleration)
{
    direction_       = (target_distance >= 0.0f) ? 1 : -1;
    target_distance_ = target_distance;
    top_speed_       = std::fabs(top_speed);
    final_speed_     = std::fabs(final_speed);
    acceleration_    = std::fabs(acceleration);
    target_speed_    = direction_ * top_speed_;

    current_velocity_     = std::fabs(start_speed) * direction_;
    current_acceleration_ = 0.0f;
    current_position_     = 0.0f;

    if (std::fabs(target_distance_) <= 0.0f || acceleration_ <= 0.0f)
    {
        state_ = State::Finished;
        return;
    }

    state_ = State::Accelerating;
}

float Profile::brakingDistance() const
{
    if (acceleration_ <= 0.0f)
        return 0.0f;
    float v_sq = current_velocity_ * current_velocity_;
    float f_sq = final_speed_ * final_speed_;
    float diff = v_sq - f_sq;
    return (diff > 0.0f) ? diff / (2.0f * acceleration_) : 0.0f;
}

float Profile::remaining() const
{
    return std::fabs(target_distance_ - current_position_);
}

void Profile::update()
{
    if (state_ == State::Idle)
        return;

    const float delta_v           = acceleration_ * LOOP_INTERVAL_S;
    const float remaining_dist    = remaining();
    const float previous_position = current_position_;

    if (state_ != State::Finished && state_ == State::Accelerating &&
        remaining_dist <= brakingDistance())
    {
        state_ = State::Braking;
    }

    if (state_ == State::Finished)
    {
        target_speed_ = direction_ * final_speed_;
    }
    else if (state_ == State::Braking)
    {
        const float finish_speed = (final_speed_ == 0.0f) ? 5.0f : final_speed_;
        target_speed_            = direction_ * finish_speed;
    }
    else
    {
        target_speed_ = direction_ * top_speed_;
    }

    const float old_velocity = current_velocity_;

    // Single-axis approach toward target_speed_; implicit cruise when v == target.
    if (current_velocity_ < target_speed_)
    {
        current_velocity_ += delta_v;
        if (current_velocity_ > target_speed_)
            current_velocity_ = target_speed_;
    }
    else if (current_velocity_ > target_speed_)
    {
        current_velocity_ -= delta_v;
        if (current_velocity_ < target_speed_)
            current_velocity_ = target_speed_;
    }

    current_acceleration_ = (current_velocity_ - old_velocity) * LOOP_FREQUENCY_HZ;
    current_position_ += current_velocity_ * LOOP_INTERVAL_S;

    const bool crossed_target = (direction_ > 0 && previous_position <= target_distance_ &&
                                 current_position_ >= target_distance_) ||
                                (direction_ < 0 && previous_position >= target_distance_ &&
                                 current_position_ <= target_distance_);

    if (state_ != State::Finished && (remaining() < 0.125f || crossed_target))
    {
        current_position_     = target_distance_;
        current_velocity_     = direction_ * final_speed_;
        current_acceleration_ = 0.0f;
        target_speed_         = direction_ * final_speed_;
        state_                = State::Finished;
    }
}

void Profile::reset()
{
    state_                = State::Idle;
    direction_            = 1;
    target_distance_      = 0.0f;
    target_speed_         = 0.0f;
    top_speed_            = 0.0f;
    final_speed_          = 0.0f;
    acceleration_         = 0.0f;
    current_velocity_     = 0.0f;
    current_acceleration_ = 0.0f;
    current_position_     = 0.0f;
}

void Profile::setTargetSpeed(float speed)
{
    top_speed_ = std::fabs(speed);
}

void Profile::setFinalSpeed(float speed)
{
    final_speed_ = std::fabs(speed);
}

void Profile::extendTarget(float distance)
{
    target_distance_ += direction_ * std::fabs(distance);
    if (state_ == State::Finished)
        state_ = State::Accelerating;
}

void Profile::setPosition(float position)
{
    current_position_ = position;
}

void Profile::adjustPosition(float delta)
{
    current_position_ += delta;
    if (state_ == State::Finished && remaining() > 0.125f)
        state_ = State::Accelerating;
}
