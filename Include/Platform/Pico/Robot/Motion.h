/******************************************************************************
 * @file    Motion.h
 * @brief   Motion manager for forward and rotational movements.
 *
 * This class manages trapezoidal velocity profiles for both forward and
 * rotational motion. Profiles provide smooth velocity setpoints, while
 * odometry/IMU provide actual state feedback. Motion ensures that moves and
 * turns complete based on IMU/encoder feedback.
 ******************************************************************************/

#ifndef MOTION_H
#define MOTION_H

#include "Drivetrain.h"
#include "Profile.h"
#include <functional>   // for std::function

/**
 * @class Motion
 * @brief High-level motion control using profiles + odometry/IMU.
 */
class Motion {
 public:
  explicit Motion(Drivetrain* drivetrain);

  // ---------------- System Control ----------------
  void resetDriveSystem();
  void stop();
  void disableDrive();

  // ---------------- Forward Motion ----------------
  void startForward(float distance_mm, float top_speed, float final_speed,
                    float accel);
  void forward(float distance_mm, float top_speed, float final_speed,
               float accel, bool blocking);
  bool isForwardFinished() const;

  // ---------------- Rotational Motion ----------------
  void startTurn(float angle_deg, float omega, float final_omega, float alpha);
  void turn(float angle_deg, float omega, float final_omega, float alpha,
            bool blocking);
  bool isTurnFinished() const;

  // ---------------- Integrated Turns ----------------
  void integratedTurn(float angle_deg, float omega, float alpha);
  void spinTurn(float angle_deg, float omega, float alpha);

  // ---------------- Stopping Utilities ----------------
  void stopAt(float target_mm);
  void stopAfter(float distance_mm);
  void waitUntilPosition(float target_mm);
  void waitUntilDistance(float delta_mm);

  // ---------------- Update Loop ----------------
  void update();

  // ---------------- State Accessors ----------------
  float positionMM() const { return drivetrain_->getOdometry()->getDistanceMM(); }
  float velocityMMPerSec() const { return drivetrain_->getOdometry()->getVelocityMMPerSec(); }
  float angleDeg() const { return drivetrain_->getOdometry()->getAngleDeg(); }
  float omegaDegPerSec() const { return drivetrain_->getOdometry()->getAngularVelocityDegPerSec(); }
  float accelerationMMPerSec2() const { return forward_profile_.acceleration(); }

  // ---------------- Wall Update Hook ----------------
  /**
   * @brief Set a callback that will be invoked when the robot reaches
   *        DEPTHINTOCELL mm into a new cell. The callback receives
   *        (left, front, right) wall existence flags.
   */
  void setWallUpdateCallback(std::function<void(bool, bool, bool)> cb) {
    wallCallback_ = cb;
  }

 private:
  Drivetrain* drivetrain_;         ///< Pointer to drivetrain
  Profile forward_profile_;        ///< Profile for forward motion
  Profile rotation_profile_;       ///< Profile for rotational motion
  float target_angle_deg_;         ///< Absolute IMU target angle for turns

  // For wall update trigger
  std::function<void(bool, bool, bool)> wallCallback_; ///< callback for wall update
  bool wallTriggeredThisCell_ = false;                 ///< track per-cell trigger
};

#endif