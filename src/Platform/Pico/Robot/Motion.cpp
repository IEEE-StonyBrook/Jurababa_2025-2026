/******************************************************************************
 * @file    Motion.cpp
 * @brief   Implementation of Motion class.
 ******************************************************************************/

#include "../../../Include/Platform/Pico/Robot/Motion.h"

#include <cmath>
#include <string>

#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/CommandHub.h"
#include "../../../Include/Platform/Pico/MulticoreSensors.h"

namespace {
/**
 * @brief Wrap an angle into [-180, 180] range for error calculation.
 */
inline float Wrap180(float angle_deg) {
  if (angle_deg > 180.0f) angle_deg -= 360.0f;
  if (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}
}  // namespace

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Motion::Motion(Drivetrain* drivetrain)
    : drivetrain_(drivetrain), target_angle_deg_(0.0f) {}

// -----------------------------------------------------------------------------
// System control
// -----------------------------------------------------------------------------
void Motion::resetDriveSystem() {
  LOG_DEBUG("Resetting drive system...");
  drivetrain_->stop();
  drivetrain_->reset();
  forward.reset();
  rotation.reset();
  LOG_DEBUG("Drive system reset complete.");
}

void Motion::stop() {
  drivetrain_->stop();
  forward.reset();
  rotation.reset();
}

void Motion::disableDrive() { drivetrain_->stop(); }

// -----------------------------------------------------------------------------
// Forward motion
// -----------------------------------------------------------------------------
void Motion::start_move(float distance, float top_speed, float final_speed,
                        float acceleration) {
  forward.start(distance, top_speed, final_speed, acceleration);
}

bool Motion::move_finished() { return forward.is_finished(); }

void Motion::move(float distance, float top_speed, float final_speed,
                  float acceleration) {
  forward.move(distance, top_speed, final_speed, acceleration);
}

void Motion::start_turn(float distance, float top_speed, float final_speed,
                        float acceleration) {
  rotation.start(distance, top_speed, final_speed, acceleration);
}

bool Motion::turn_finished() { return rotation.is_finished(); }

void Motion::turn(float distance, float top_speed, float final_speed,
                  float acceleration) {
  rotation.move(distance, top_speed, final_speed, acceleration);
}

// -----------------------------------------------------------------------------
// Update loop
// -----------------------------------------------------------------------------
void Motion::update() {
  forward.update();
  rotation.update();
}

// Example Usages
//***************************************************************************//

/**
 * Performs a turn. Regardless of whether the robot is moving or not
 *
 * The function is given three parameters
 *
 *  - angle  : positive is a left turn (deg)
 *  - omega  : angular velocity of middle phase (deg/s)
 *  - alpha  : angular acceleration of in/out phases (deg/s/s)
 *
 * If the robot is moving forward, it will execute a smooth, integrated
 * turn. The turn will only be repeatable if it is always performed at the
 * same forward speed.
 *
 * If the robot is stationary, it will execute an in-place spin turn.
 *
 * The parameter alpha will indirectly determine the turn radius. During
 * the accelerating phase, the angular velocity, will increase until it
 * reaches the value omega.
 * The minimum radius during the constant phase is
 *   radius = (speed/omega) * (180/PI)
 * The effective radius will be larger because it takes some time
 * for the rotation to accelerate and decelerate. The parameter alpha
 * controls that.
 *
 * Note that a real mouse may behave slightly different for left and
 * right turns and so the parameters for, say, a 90 degree left turn
 * may be slightly different to those for a 90 degree right turn.
 *
 * @brief execute an arbitrary in-place or smooth turn
 */
void Motion::turn(float angle, float omega, float alpha) {
  // get ready to turn
  rotation.reset();
  rotation.move(angle, omega, 0, alpha);
}

/**
 *
 * @brief turn in place. Force forward speed to zero
 */
void Motion::spin_turn(float angle, float omega, float alpha) {
  forward.set_target_speed(0);
  while (forward.speed() != 0) {
    sleep_ms(2000);
  }
  turn(angle, omega, alpha);
};

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
void Motion::stop_at(float position) {
  float remaining = position - forward.position();
  forward.move(remaining, forward.speed(), 0, forward.acceleration());
}

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
void Motion::stop_after(float distance) {
  forward.move(distance, forward.speed(), 0, forward.acceleration());
}

/**
 * The robot is assumed to be moving. This utility function call will just
 * do a busy-wait until the forward profile gets to the supplied position.
 *
 * @brief wait until the given position is reached
 */
void Motion::wait_until_position(float position) {
  while (forward.position() < position) {
    sleep_ms(2000);
  }
}

/**
 * The robot is assumed to be moving. This utility function call will just
 * do a busy-wait until the forward profile has moved by the given distance.
 *
 * @brief wait until the given distance has been travelled
 */
void Motion::wait_until_distance(float distance) {
  float target = forward.position() + distance;
  wait_until_position(target);
}