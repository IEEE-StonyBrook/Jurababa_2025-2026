#ifndef MOTION_H
#define MOTION_H

#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../../../Include/Platform/Pico/Robot/Profile.h"

/**
 * Motion converts planner commands into smooth forward and rotational movement
 * using trapezoidal profiles and the drivetrain.
 */
class Motion {
 public:
  explicit Motion(Drivetrain* drivetrain);

  // Reset drivetrain and motion profiles.
  void resetDriveSystem();

  // Stop all motion immediately.
  void stop();
  void disableDrive();

  // Forward state access.
  inline float positionMM() const { return drivetrain->getOdometry()->getDistanceMM(); }
  inline float velocityMMPerSec() const { return drivetrain->getOdometry()->getVelocityMMPerSec(); }
  inline float accelerationMMPerSec2() const { return forwardProfile.acceleration(); }

  // Rotation state access.
  inline float angleDeg() const { return drivetrain->getOdometry()->getAngleDeg(); }
  inline float omegaDegPerSec() const { return drivetrain->getOdometry()->getAngularVelocityDegPerSec(); }
  inline float alphaDegPerSec2() const { return rotationProfile.acceleration(); }

  // Forward moves.
  void startForward(float distanceMM, float topSpeed, float finalSpeed, float accel);
  void forward(float distanceMM, float topSpeed, float finalSpeed, float accel, bool blocking = true);
  bool isForwardFinished() const;

  // Rotational moves.
  void startTurn(float angleDeg, float omega, float finalOmega, float alpha);
  void turn(float angleDeg, float omega, float finalOmega, float alpha, bool blocking = true);
  bool isTurnFinished() const;

  // Integrated and spin turns.
  void integratedTurn(float angleDeg, float omega, float alpha);
  void spinTurn(float angleDeg, float omega, float alpha);

  // Stopping utilities.
  void stopAt(float positionMM);
  void stopAfter(float distanceMM);
  void waitUntilPosition(float targetMM);
  void waitUntilDistance(float deltaMM);

  // Update profiles and apply control.
  void update();

 private:
  Drivetrain* drivetrain;
  Profile forwardProfile;
  Profile rotationProfile;
};

#endif