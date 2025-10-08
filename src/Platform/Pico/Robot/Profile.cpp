#include "../../../Include/Platform/Pico/Robot/Profile.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/Config.h"

Profile::Profile()
    : state_(State::Idle),
      speed_(0.0f),
      targetSpeed_(0.0f),
      finalSpeed_(0.0f),
      distance_(0.0f),
      startPosition_(0.0f),
      acceleration_(0.0f),
      invAcceleration_(1.0f),
      directionSign_(1) {}

void Profile::reset() {
  state_ = State::Idle;
  speed_ = 0.0f;
  targetSpeed_ = 0.0f;
  finalSpeed_ = 0.0f;
  distance_ = 0.0f;
  startPosition_ = 0.0f;
  acceleration_ = 0.0f;
  invAcceleration_ = 1.0f;
  directionSign_ = 1;
}

void Profile::start(float distance, float maxSpeed, float finalSpeed,
                    float acceleration, float startPos) {
  directionSign_ = (distance < 0) ? -1 : 1;
  distance = std::fabs(distance);

  if (distance < 1e-3f) {
    state_ = State::Finished;
    return;
  }

  if (finalSpeed > maxSpeed) {
    finalSpeed = maxSpeed;
  }

  distance_ = distance;
  startPosition_ = startPos;

  targetSpeed_ = directionSign_ * std::fabs(maxSpeed);
  finalSpeed_ = directionSign_ * std::fabs(finalSpeed);
  acceleration_ = std::fabs(acceleration);
  invAcceleration_ = (acceleration_ >= 1e-6f) ? (1.0f / acceleration_) : 1.0f;

  speed_ = 0.0f;
  state_ = State::Accelerating;
}

void Profile::update(float currentPos) {
  if (state_ == State::Idle || state_ == State::Finished) return;

  float deltaV = acceleration_ * LOOP_INTERVAL_S;
  float remaining = remainingDistance(currentPos);

  // Transition to braking if needed
  if (state_ == State::Accelerating && remaining < brakingDistance()) {
    state_ = State::Braking;
    targetSpeed_ = finalSpeed_;
    LOG_DEBUG("Switching to braking phase.");
  }

  // Adjust speed toward target
  if (speed_ < targetSpeed_) {
    speed_ = std::min(speed_ + deltaV, targetSpeed_);
  } else if (speed_ > targetSpeed_) {
    speed_ = std::max(speed_ - deltaV, targetSpeed_);
  }

  // Finish when close enough
  if (remaining < 1e-2f) {
    state_ = State::Finished;
    speed_ = finalSpeed_;
    LOG_DEBUG("Profile finished. Target reached.");
  }
}

void Profile::stop() {
  targetSpeed_ = 0.0f;
  speed_ = 0.0f;
  state_ = State::Finished;
}

bool Profile:: isFinished() const { return state_ == State::Finished; }

float Profile::speed() const { return speed_; }

float Profile::acceleration() const { return acceleration_; }

float Profile::remainingDistance(float currentPos) const {
  float traveled = std::fabs(currentPos - startPosition_);
  return std::max(0.0f, distance_ - traveled);
}

float Profile::brakingDistance() const {
  return std::fabs(speed_ * speed_ - finalSpeed_ * finalSpeed_) * 0.5f *
         invAcceleration_;
}