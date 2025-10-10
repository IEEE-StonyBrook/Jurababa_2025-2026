#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Encoder.h"
#include "IMU.h"
#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/Config.h"

class Odometry {
 public:
  Odometry(Encoder* leftEncoder, Encoder* rightEncoder, IMU* imu);

  void reset();
  void update();

  // Robot-level metrics
  float getDistanceMM() const;
  float getVelocityMMPerSec() const;
  float getAngleDeg() const;
  float getAngularVelocityDegPerSec() const;
  float getDeltaDistanceMM() const;
  float getDeltaAngleDeg() const;

  // Wheel-level metrics
  float getLeftWheelVelocityMMPerSec() const;
  float getRightWheelVelocityMMPerSec() const;
  float getLeftDeltaMM() const;
  float getRightDeltaMM() const;

 private:
  Encoder* leftEncoder;
  Encoder* rightEncoder;
  IMU* imu;

  int lastLeftTicks;
  int lastRightTicks;
  float lastAngleDeg;

  // Robot-level
  float totalDistanceMM;
  float totalAngleDeg;
  float deltaDistanceMM;
  float deltaAngleDeg;

  // Wheel-level
  float deltaLeftMM;
  float deltaRightMM;
};

#endif