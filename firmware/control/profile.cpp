#include "control/profile.h"
#include <cmath>

static constexpr float DEFAULT_POSITION_TOLERANCE = 2.0f;
static constexpr float DEFAULT_VELOCITY_TOLERANCE = 30.0f;

Profile::Profile()
    : state_(State::Idle),
      target_distance_(0.0f),
      max_velocity_(0.0f),
      acceleration_(0.0f),
      initial_position_(0.0f),
      current_velocity_(0.0f),
      current_acceleration_(0.0f),
      accel_distance_(0.0f),
      decel_distance_(0.0f),
      cruise_distance_(0.0f),
      cruise_velocity_(0.0f),
      position_tolerance_(DEFAULT_POSITION_TOLERANCE),
      velocity_tolerance_(DEFAULT_VELOCITY_TOLERANCE)
{
}

void Profile::start(float target_distance, float max_velocity, float acceleration,
                    float initial_position)
{
    target_distance_  = std::fabs(target_distance);
    max_velocity_     = std::fabs(max_velocity);
    acceleration_     = std::fabs(acceleration);
    initial_position_ = initial_position;

    current_velocity_     = 0.0f;
    current_acceleration_ = 0.0f;

    calculateProfileParameters();

    if (target_distance_ > position_tolerance_)
    {
        state_               = State::Accelerating;
        current_acceleration_ = acceleration_;
    }
    else
    {
        state_ = State::Finished;
    }
}

void Profile::calculateProfileParameters()
{
    float dist_to_max_vel = (max_velocity_ * max_velocity_) / (2.0f * acceleration_);

    if (2.0f * dist_to_max_vel <= target_distance_)
    {
        accel_distance_ = dist_to_max_vel;
        decel_distance_ = dist_to_max_vel;
        cruise_distance_ = target_distance_ - (accel_distance_ + decel_distance_);
        cruise_velocity_ = max_velocity_;
    }
    else
    {
        accel_distance_  = target_distance_ / 2.0f;
        decel_distance_  = target_distance_ / 2.0f;
        cruise_distance_ = 0.0f;
        cruise_velocity_ = std::sqrt(2.0f * acceleration_ * accel_distance_);
    }
}

void Profile::update(float current_position, float dt)
{
    if (state_ == State::Idle || state_ == State::Finished)
        return;

    float distance_traveled = std::fabs(current_position - initial_position_);
    updateState(distance_traveled);

    switch (state_)
    {
        case State::Accelerating:
            current_velocity_ += acceleration_ * dt;
            if (current_velocity_ >= cruise_velocity_)
            {
                current_velocity_ = cruise_velocity_;
            }
            current_acceleration_ = acceleration_;
            break;

        case State::Cruising:
            current_velocity_     = cruise_velocity_;
            current_acceleration_ = 0.0f;
            break;

        case State::Decelerating:
            current_velocity_ -= acceleration_ * dt;
            if (current_velocity_ <= 0.0f)
            {
                current_velocity_ = 0.0f;
            }
            current_acceleration_ = -acceleration_;
            break;

        case State::Finished:
            current_velocity_     = 0.0f;
            current_acceleration_ = 0.0f;
            break;

        case State::Idle:
            break;
    }
}

void Profile::updateState(float distance_traveled)
{
    float remaining_dist = target_distance_ - distance_traveled;

    if (remaining_dist <= position_tolerance_ && current_velocity_ <= velocity_tolerance_)
    {
        state_ = State::Finished;
        return;
    }

    if (distance_traveled < accel_distance_)
    {
        state_ = State::Accelerating;
    }
    else if (distance_traveled < (accel_distance_ + cruise_distance_))
    {
        state_ = State::Cruising;
    }
    else
    {
        state_ = State::Decelerating;
    }

    float braking_distance = (current_velocity_ * current_velocity_) / (2.0f * acceleration_);

    if (remaining_dist <= braking_distance && state_ != State::Finished)
    {
        state_ = State::Decelerating;
    }
}

float Profile::velocity() const
{
    return current_velocity_;
}

float Profile::acceleration() const
{
    return current_acceleration_;
}

float Profile::remaining() const
{
    if (state_ == State::Idle || state_ == State::Finished)
        return 0.0f;

    return target_distance_ * (current_velocity_ / cruise_velocity_);
}

bool Profile::finished() const
{
    return (state_ == State::Finished);
}

Profile::State Profile::state() const
{
    return state_;
}

void Profile::reset()
{
    state_               = State::Idle;
    target_distance_      = 0.0f;
    max_velocity_         = 0.0f;
    acceleration_        = 0.0f;
    initial_position_     = 0.0f;
    current_velocity_     = 0.0f;
    current_acceleration_ = 0.0f;
    accel_distance_       = 0.0f;
    decel_distance_       = 0.0f;
    cruise_distance_      = 0.0f;
    cruise_velocity_      = 0.0f;
}
