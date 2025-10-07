#include "../../../Include/Platform/Pico/Robot/Profile.h"

Profile::Profile()
    : state(State::Idle),
      positionMM(0.0f),
      speedMMPerSec(0.0f),
      targetSpeedMMPerSec(0.0f),
      finalSpeedMMPerSec(0.0f),
      finalPositionMM(0.0f),
      accelerationMMPerSec2(0.0f),
      invAcceleration(1.0f),
      directionSign(1) {}

void Profile::reset() {
  state = State::Idle;
  positionMM = 0.0f;
  speedMMPerSec = 0.0f;
  targetSpeedMMPerSec = 0.0f;
  finalSpeedMMPerSec = 0.0f;
  finalPositionMM = 0.0f;
  accelerationMMPerSec2 = 0.0f;
  invAcceleration = 1.0f;
  directionSign = 1;
}

void Profile::start(float distance, float maxSpeed, float finalSpeed, float acceleration) {
  directionSign = (distance < 0) ? -1 : 1;
  distance = std::fabs(distance);

  if (distance < 1.0f) {
    state = State::Finished;
    return;
  }

  if (finalSpeed > maxSpeed) {
    finalSpeed = maxSpeed;
  }

  positionMM = 0.0f;
  finalPositionMM = distance;
  targetSpeedMMPerSec = directionSign * std::fabs(maxSpeed);
  finalSpeedMMPerSec = directionSign * std::fabs(finalSpeed);
  accelerationMMPerSec2 = std::fabs(acceleration);
  invAcceleration = (accelerationMMPerSec2 >= 1e-6f) ? (1.0f / accelerationMMPerSec2) : 1.0f;

  state = State::Accelerating;
}

void Profile::move(float distance, float maxSpeed, float finalSpeed, float acceleration) {
  start(distance, maxSpeed, finalSpeed, acceleration);
  while (!isFinished()) {
    update();
  }
}

void Profile::stop() {
  targetSpeedMMPerSec = 0.0f;
  finish();
}

void Profile::finish() {
  speedMMPerSec = targetSpeedMMPerSec;
  state = State::Finished;
}

void Profile::update() {
  if (state == State::Idle || state == State::Finished) return;

  float deltaV = accelerationMMPerSec2 * LOOP_INTERVAL_S;
  float remaining = std::fabs(finalPositionMM) - std::fabs(positionMM);

  if (state == State::Accelerating && remaining < brakingDistance()) {
    state = State::Braking;
    targetSpeedMMPerSec = finalSpeedMMPerSec;
  }

  // Adjust speed toward target.
  if (speedMMPerSec < targetSpeedMMPerSec) {
    speedMMPerSec = std::min(speedMMPerSec + deltaV, targetSpeedMMPerSec);
  } else if (speedMMPerSec > targetSpeedMMPerSec) {
    speedMMPerSec = std::max(speedMMPerSec - deltaV, targetSpeedMMPerSec);
  }

  // Update position.
  positionMM += speedMMPerSec * LOOP_INTERVAL_S;

  // End condition (close enough).
  if (state != State::Finished && remaining < 0.1f) {
    state = State::Finished;
    targetSpeedMMPerSec = finalSpeedMMPerSec;
  }
}

bool Profile::isFinished() const {
  return state == State::Finished;
}

float Profile::position() const {
  return positionMM;
}

float Profile::speed() const {
  return speedMMPerSec;
}

float Profile::acceleration() const {
  return accelerationMMPerSec2;
}

void Profile::setSpeed(float speed) {
  speedMMPerSec = speed;
}

void Profile::setTargetSpeed(float targetSpeed) {
  targetSpeedMMPerSec = targetSpeed;
}

void Profile::setPosition(float position) {
  positionMM = position;
}

void Profile::adjustPosition(float delta) {
  positionMM += delta;
}

float Profile::brakingDistance() const {
  return std::fabs(speedMMPerSec * speedMMPerSec - finalSpeedMMPerSec * finalSpeedMMPerSec) * 0.5f * invAcceleration;
}