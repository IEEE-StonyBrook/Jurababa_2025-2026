#include "control/profile.h"

#include <cmath>

#include "config/motion.h"

Profile::Profile()
    : state_(State::Idle), direction_(1), target_distance_(0.0f), top_speed_(0.0f),
      final_speed_(0.0f), acceleration_(0.0f), current_velocity_(0.0f), current_acceleration_(0.0f),
      current_position_(0.0f)
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
    target_distance_ = std::fabs(target_distance);
    top_speed_       = std::fabs(top_speed);
    final_speed_     = std::fabs(final_speed);
    acceleration_    = std::fabs(acceleration);

    current_velocity_     = std::fabs(start_speed) * direction_;
    current_acceleration_ = 0.0f;
    current_position_     = 0.0f;

    if (target_distance_ <= 0.0f || acceleration_ <= 0.0f)
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
    float traveled = std::fabs(current_position_);
    float r        = target_distance_ - traveled;
    return (r > 0.0f) ? r : 0.0f;
}

void Profile::update()
{
    if (state_ == State::Idle || state_ == State::Finished)
        return;

    float delta_v        = acceleration_ * LOOP_INTERVAL_S;
    float remaining_dist = target_distance_ - std::fabs(current_position_);

    // Pick target speed for this tick: braking phase decelerates toward
    // final_speed; otherwise we keep climbing toward top_speed.
    float target_speed = top_speed_;
    if (state_ == State::Accelerating && remaining_dist <= brakingDistance())
    {
        state_       = State::Braking;
        target_speed = final_speed_;
    }
    else if (state_ == State::Braking)
    {
        target_speed = final_speed_;
    }

    float signed_target = target_speed * direction_;

    // Single-axis approach toward signed_target — implicit cruise when v == target.
    if (current_velocity_ < signed_target)
    {
        current_velocity_ += delta_v;
        if (current_velocity_ > signed_target)
            current_velocity_ = signed_target;
    }
    else if (current_velocity_ > signed_target)
    {
        current_velocity_ -= delta_v;
        if (current_velocity_ < signed_target)
            current_velocity_ = signed_target;
    }

    current_acceleration_ =
        (state_ == State::Braking) ? -acceleration_ * direction_ : acceleration_ * direction_;
    if (current_velocity_ == signed_target)
        current_acceleration_ = 0.0f;

    current_position_ += current_velocity_ * LOOP_INTERVAL_S;

    if (remaining_dist < 0.125f)
        state_ = State::Finished;
}

void Profile::reset()
{
    state_                = State::Idle;
    direction_            = 1;
    target_distance_      = 0.0f;
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

void Profile::setPosition(float position)
{
    current_position_ = position;
}

void Profile::adjustPosition(float delta)
{
    current_position_ += delta;
}
