#include "../../../Include/Platform/Pico/Robot/Odometry.h"

// Constructor initializes encoder references and resets state.
Odometry::Odometry(Encoder* leftEncoder, Encoder* rightEncoder)
    : leftEncoder(leftEncoder),
      rightEncoder(rightEncoder),
      lastLeftTicks(0),
      lastRightTicks(0),
      totalDistanceMM(0.0f),
      totalAngleDeg(0.0f),
      deltaDistanceMM(0.0f),
      deltaAngleDeg(0.0f) {}

// Reset odometry to zero.
void Odometry::reset() {
  leftEncoder->reset();
  rightEncoder->reset();
  lastLeftTicks = 0;
  lastRightTicks = 0;
  totalDistanceMM = 0.0f;
  totalAngleDeg = 0.0f;
  deltaDistanceMM = 0.0f;
  deltaAngleDeg = 0.0f;
}

// Update odometry using encoder tick changes.
void Odometry::update() {
  int currentLeftTicks = leftEncoder->getTickCount();
  int currentRightTicks = rightEncoder->getTickCount();

  int deltaLeftTicks = currentLeftTicks - lastLeftTicks;
  int deltaRightTicks = currentRightTicks - lastRightTicks;

  lastLeftTicks = currentLeftTicks;
  lastRightTicks = currentRightTicks;

  float deltaLeftMM = deltaLeftTicks * MM_PER_TICK;
  float deltaRightMM = deltaRightTicks * MM_PER_TICK;

  deltaDistanceMM = 0.5f * (deltaLeftMM + deltaRightMM);
  deltaAngleDeg = (deltaRightMM - deltaLeftMM) * DEG_PER_MM_DIFFERENCE;

  totalDistanceMM += deltaDistanceMM;
  totalAngleDeg += deltaAngleDeg;
}

// Return total forward distance in millimeters.
float Odometry::getDistanceMM() const { return totalDistanceMM; }

// Return forward velocity in millimeters per second.
float Odometry::getVelocityMMPerSec() const {
  return deltaDistanceMM * LOOP_FREQUENCY_HZ;
}

// Return robot heading angle in degrees.
float Odometry::getAngleDeg() const { return totalAngleDeg; }

// Return angular velocity in degrees per second.
float Odometry::getAngularVelocityDegPerSec() const {
  return deltaAngleDeg * LOOP_FREQUENCY_HZ;
}

// Return forward distance change since last update in millimeters.
float Odometry::getDeltaDistanceMM() const { return deltaDistanceMM; }

// Return rotation change since last update in degrees.
float Odometry::getDeltaAngleDeg() const { return deltaAngleDeg; }