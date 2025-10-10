#include "../../../Include/Platform/Pico/Robot/Odometry.h"

// Constructor initializes encoder references and resets state.
Odometry::Odometry(Encoder* leftEncoder, Encoder* rightEncoder, IMU* imu)
    : leftEncoder(leftEncoder),
      rightEncoder(rightEncoder),
      imu(imu),
      lastLeftTicks(0),
      lastRightTicks(0),
      totalDistanceMM(0.0f),
      totalAngleDeg(0.0f),
      deltaDistanceMM(0.0f),
      deltaAngleDeg(0.0f) {}

// Reset odometry to zero.
void Odometry::reset() {
  LOG_DEBUG("[Odometry] Resetting encoders...");
  leftEncoder->reset();
  rightEncoder->reset();

  LOG_DEBUG("[Odometry] Resetting IMU yaw to zero...");
  imu->resetIMUYawToZero();

  LOG_DEBUG("[Odometry] Clearing internal state...");
  lastLeftTicks = 0;
  lastRightTicks = 0;
  lastAngleDeg = 0.0f;
  totalDistanceMM = 0.0f;
  totalAngleDeg = 0.0f;
  deltaDistanceMM = 0.0f;
  deltaAngleDeg = 0.0f;
}

// Update odometry using encoder tick changes and IMU angle.
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

  float currentAngleDeg = imu->getIMUYawDegreesNeg180ToPos180();
  deltaAngleDeg = currentAngleDeg - lastAngleDeg;
  if (deltaAngleDeg > 180.0f) deltaAngleDeg -= 360.0f;
  if (deltaAngleDeg < -180.0f) deltaAngleDeg += 360.0f;
  lastAngleDeg = currentAngleDeg;

  totalDistanceMM += deltaDistanceMM;
  totalAngleDeg += deltaAngleDeg;

  // ---- Debug logs ----
  LOG_DEBUG("[Odometry] ΔTicks L=" + std::to_string(deltaLeftTicks) +
            " R=" + std::to_string(deltaRightTicks));
  LOG_DEBUG("[Odometry] ΔDist=" + std::to_string(deltaDistanceMM) + " mm");
  LOG_DEBUG("[Odometry] ΔAngle=" + std::to_string(deltaAngleDeg) + " deg");
  LOG_DEBUG("[Odometry] TotalDist=" + std::to_string(totalDistanceMM) +
            " mm, TotalAngle=" + std::to_string(totalAngleDeg) + " deg");
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