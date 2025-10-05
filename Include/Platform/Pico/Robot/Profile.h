#ifndef PROFILE_H
#define PROFILE_H

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../Config.h"


/**
 * @brief Motion profile generator for trapezoidal speed control.
 *
 * A profile consists of three phases:
 * - Acceleration: Ramp speed up toward max speed.
 * - Coasting: Maintain max speed if distance allows.
 * - Braking: Ramp speed down to final speed.
 *
 * The profile runs incrementally and must be updated
 * every control cycle via update().
 */
class Profile {
 public:
  enum class State : uint8_t { Idle, Accelerating, Braking, Finished };

  Profile();

  // Reset profile to idle state.
  void reset();

  // Start a new profile.
  void start(float distance, float maxSpeed, float finalSpeed,
             float acceleration);

  // Blocking move (wait until finished).
  void move(float distance, float maxSpeed, float finalSpeed,
            float acceleration);

  // Stop immediately with target speed set to zero.
  void stop();

  // Force profile to finish at final speed.
  void finish();

  // Update profile state (must be called each control cycle).
  void update();

  // Check if profile has completed.
  bool isFinished() const;

  // Getters for current motion state.
  float position() const;
  float speed() const;
  float acceleration() const;

  // Setters for corrections.
  void setSpeed(float speed);
  void setTargetSpeed(float targetSpeed);
  void setPosition(float position);
  void adjustPosition(float delta);

 private:
  float brakingDistance() const;

  State state;
  float positionMM;
  float speedMMPerSec;
  float targetSpeedMMPerSec;
  float finalSpeedMMPerSec;
  float finalPositionMM;
  float accelerationMMPerSec2;
  float invAcceleration;
  int directionSign;
};

#endif