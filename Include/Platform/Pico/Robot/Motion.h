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

#include <functional>  // for std::function

#include "Drivetrain.h"
#include "Profile.h"

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

  void start_move(float distance, float top_speed, float final_speed,
                  float acceleration);
  bool move_finished();
  void move(float distance, float top_speed, float final_speed,
            float acceleration);

  void start_turn(float distance, float top_speed, float final_speed,
                  float acceleration);
  bool turn_finished();
  void turn(float distance, float top_speed, float final_speed,
            float acceleration);

  // Example
  void turn(float angle, float omega, float alpha);

  /**
   *
   * @brief turn in place. Force forward speed to zero
   */
  void spin_turn(float angle, float omega, float alpha);

  //***************************************************************************//
  /**
   * These are examples of ways to use the motion control functions
   */

  /**
   * The robot is assumed to be moving. This call will stop at a specific
   * distance. Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt at a specific distance
   */
  void stop_at(float position);

  /**
   * The robot is assumed to be moving. This call will stop  after a
   * specific distance has been travelled
   *
   * Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt after a specific distance
   */
  void stop_after(float distance);

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile gets to the supplied position.
   *
   * @brief wait until the given position is reached
   */
  void wait_until_position(float position);

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile has moved by the given distance.
   *
   * @brief wait until the given distance has been travelled
   */
  void wait_until_distance(float distance);

  // ---------------- Update Loop ----------------
  void update();

  // ---------------- State Accessors ----------------
  float positionMM() const {
    return drivetrain_->getOdometry()->getDistanceMM();
  }
  float velocityMMPerSec() const {
    return drivetrain_->getOdometry()->getVelocityMMPerSec();
  }
  float angleDeg() const { return drivetrain_->getOdometry()->getAngleDeg(); }
  float omegaDegPerSec() const {
    return drivetrain_->getOdometry()->getAngularVelocityDegPerSec();
  }
  float accelerationMMPerSec2() const { return forward.acceleration(); }

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
  Drivetrain* drivetrain_;  ///< Pointer to drivetrain
  Profile forward;          ///< Profile for forward motion
  Profile rotation;         ///< Profile for rotational motion
  float target_angle_deg_;  ///< Absolute IMU target angle for turns

  // For wall update trigger
  std::function<void(bool, bool, bool)>
      wallCallback_;                    ///< callback for wall update
  bool wallTriggeredThisCell_ = false;  ///< track per-cell trigger
};

#endif