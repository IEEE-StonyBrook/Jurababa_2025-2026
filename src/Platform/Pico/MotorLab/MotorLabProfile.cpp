#include "Platform/Pico/MotorLab/MotorLabProfile.h"

#include <cmath>

MotorLabProfile::MotorLabProfile()
    : state_(MotorLabProfileState::IDLE),
      target_distance_(0.0f),
      top_speed_(0.0f),
      final_speed_(0.0f),
      acceleration_(0.0f),
      position_(0.0f),
      speed_(0.0f),
      direction_(1.0f) {
}

void MotorLabProfile::start(float distance, float top_speed, float acceleration, float final_speed) {
    // Determine direction
    direction_ = (distance >= 0.0f) ? 1.0f : -1.0f;
    target_distance_ = std::fabs(distance);
    top_speed_ = std::fabs(top_speed);
    final_speed_ = std::fabs(final_speed);
    acceleration_ = std::fabs(acceleration);

    // Reset runtime state
    position_ = 0.0f;
    speed_ = 0.0f;

    // Start accelerating
    state_ = MotorLabProfileState::ACCELERATING;
}

void MotorLabProfile::update(float dt) {
    if (dt <= 0.0f || state_ == MotorLabProfileState::IDLE ||
        state_ == MotorLabProfileState::FINISHED) {
        return;
    }

    // Calculate remaining distance
    float remaining = target_distance_ - position_;

    // Magic number to ensure we actually stop (from UKMARS)
    const float STOP_MARGIN = 5.0f;

    switch (state_) {
        case MotorLabProfileState::ACCELERATING:
            // Increase speed toward top_speed
            speed_ += acceleration_ * dt;
            if (speed_ >= top_speed_) {
                speed_ = top_speed_;
                state_ = MotorLabProfileState::CRUISING;
            }
            // Check if we need to start braking
            if (remaining <= getBrakingDistance() + STOP_MARGIN) {
                state_ = MotorLabProfileState::BRAKING;
            }
            break;

        case MotorLabProfileState::CRUISING:
            // Maintain constant speed
            speed_ = top_speed_;
            // Check if we need to start braking
            if (remaining <= getBrakingDistance() + STOP_MARGIN) {
                state_ = MotorLabProfileState::BRAKING;
            }
            break;

        case MotorLabProfileState::BRAKING:
            // Decrease speed toward final_speed
            speed_ -= acceleration_ * dt;
            if (speed_ <= final_speed_) {
                speed_ = final_speed_;
            }
            // Check if we've reached target
            if (remaining <= 0.0f || (final_speed_ == 0.0f && speed_ <= 0.0f)) {
                speed_ = final_speed_;
                position_ = target_distance_;
                state_ = MotorLabProfileState::FINISHED;
            }
            break;

        case MotorLabProfileState::IDLE:
        case MotorLabProfileState::FINISHED:
            // Nothing to do
            return;
    }

    // Update position based on current speed
    position_ += speed_ * dt;

    // Clamp position to target
    if (position_ >= target_distance_) {
        position_ = target_distance_;
        if (state_ != MotorLabProfileState::FINISHED) {
            state_ = MotorLabProfileState::FINISHED;
            speed_ = final_speed_;
        }
    }
}

float MotorLabProfile::getBrakingDistance() const {
    // Distance = (v² - v_final²) / (2 * a)
    // This is the standard kinematic equation for deceleration
    float delta_v_squared = speed_ * speed_ - final_speed_ * final_speed_;
    if (delta_v_squared <= 0.0f || acceleration_ <= 0.0f) {
        return 0.0f;
    }
    return delta_v_squared / (2.0f * acceleration_);
}

float MotorLabProfile::getAcceleration() const {
    switch (state_) {
        case MotorLabProfileState::ACCELERATING:
            return direction_ * acceleration_;
        case MotorLabProfileState::BRAKING:
            return -direction_ * acceleration_;
        default:
            return 0.0f;
    }
}

void MotorLabProfile::reset() {
    state_ = MotorLabProfileState::IDLE;
    position_ = 0.0f;
    speed_ = 0.0f;
    target_distance_ = 0.0f;
}
