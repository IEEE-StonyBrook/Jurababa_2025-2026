#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "../../../Include/Platform/Pico/config.h"
#include "Encoder.h"

class Odometry {
 public:
  // Constructor initializes with references to left and right encoders.
  Odometry(Encoder* leftEncoder, Encoder* rightEncoder);

  // Reset all odometry values to zero.
  void reset();

  // Update odometry values. Must be called once per control loop.
  void update();

  // Return total forward distance traveled in millimeters.
  float getDistanceMM() const;

  // Return current forward velocity in millimeters per second.
  float getVelocityMMPerSec() const;

  // Return current robot heading angle in degrees.
  float getAngleDeg() const;

  // Return angular velocity in degrees per second.
  float getAngularVelocityDegPerSec() const;

  // Return forward distance change since last update in millimeters.
  float getDeltaDistanceMM() const;

  // Return rotation change since last update in degrees.
  float getDeltaAngleDeg() const;

 private:
  Encoder* leftEncoder;
  Encoder* rightEncoder;

  int lastLeftTicks;
  int lastRightTicks;

  float totalDistanceMM;
  float totalAngleDeg;
  float deltaDistanceMM;
  float deltaAngleDeg;
};

#endif