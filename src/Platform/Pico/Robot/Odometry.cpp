#include "../../../Include/Platform/Pico/Robot/Odometry.h"

// Constructor initializes encoder references and resets state.
Odometry::Odometry(Encoder* leftEncoder, Encoder* rightEncoder, IMU* imu)
    : leftEncoder(leftEncoder),
      rightEncoder(rightEncoder),
      imu(imu),
      lastLeftTicks(0),
      lastRightTicks(0),
      lastAngleDeg(0.0f),
      totalDistanceMM(0.0f),
      totalAngleDeg(0.0f),
      deltaDistanceMM(0.0f),
      deltaAngleDeg(0.0f),
      deltaLeftMM(0.0f),
      deltaRightMM(0.0f) {}

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
  deltaLeftMM = 0.0f;
  deltaRightMM = 0.0f;
}

// Update odometry using encoder tick changes and IMU angle.
void Odometry::update() {
  int currentLeftTicks = leftEncoder->getTickCount();
  int currentRightTicks = rightEncoder->getTickCount();

  int deltaLeftTicks = currentLeftTicks - lastLeftTicks;
  int deltaRightTicks = currentRightTicks - lastRightTicks;

  lastLeftTicks = currentLeftTicks;
  lastRightTicks = currentRightTicks;

  // Per-wheel deltas
  deltaLeftMM = deltaLeftTicks * MM_PER_TICK;
  deltaRightMM = deltaRightTicks * MM_PER_TICK;

  // Robot forward delta (average of wheels)
  deltaDistanceMM = 0.5f * (deltaLeftMM + deltaRightMM);

  // Robot rotation from IMU
  float currentAngleDeg = imu->getIMUYawDegreesNeg180ToPos180();
  deltaAngleDeg = currentAngleDeg - lastAngleDeg;
  if (deltaAngleDeg > 180.0f) deltaAngleDeg -= 360.0f;
  if (deltaAngleDeg < -180.0f) deltaAngleDeg += 360.0f;
  lastAngleDeg = currentAngleDeg;

  // Integrate totals
  totalDistanceMM += deltaDistanceMM;
  totalAngleDeg += deltaAngleDeg;

  // Debug
  LOG_DEBUG("[Odometry] DeltaTicks L=" + std::to_string(deltaLeftTicks) +
            " R=" + std::to_string(deltaRightTicks));
  LOG_DEBUG("[Odometry] DeltaL=" + std::to_string(deltaLeftMM) +
            " mm, DeltaR=" + std::to_string(deltaRightMM) + " mm");
  LOG_DEBUG("[Odometry] DeltaDist=" + std::to_string(deltaDistanceMM) +
            " mm, DeltaAng=" + std::to_string(deltaAngleDeg) + " deg");
  LOG_DEBUG("[Odometry] TotDist=" + std::to_string(totalDistanceMM) +
            " mm, TotAng=" + std::to_string(totalAngleDeg) + " deg");
}

// Robot-level accessors
float Odometry::getDistanceMM() const { return totalDistanceMM; }
float Odometry::getVelocityMMPerSec() const {
  return deltaDistanceMM * LOOP_FREQUENCY_HZ;
}
float Odometry::getAngleDeg() const { return totalAngleDeg; }
float Odometry::getAngularVelocityDegPerSec() const {
  return deltaAngleDeg * LOOP_FREQUENCY_HZ;
}
float Odometry::getDeltaDistanceMM() const { return deltaDistanceMM; }
float Odometry::getDeltaAngleDeg() const { return deltaAngleDeg; }

// Wheel-level accessors
float Odometry::getLeftWheelVelocityMMPerSec() const {
  return deltaLeftMM * LOOP_FREQUENCY_HZ;
}
float Odometry::getRightWheelVelocityMMPerSec() const {
  return deltaRightMM * LOOP_FREQUENCY_HZ;
}
float Odometry::getLeftDeltaMM() const { return deltaLeftMM; }
float Odometry::getRightDeltaMM() const { return deltaRightMM; }
