/******************************************************************************
 * @file    Motion.cpp
 * @brief   Implementation of Motion class with debug logging.
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
  LOG_DEBUG("[Motion] Resetting drive system...");
  drivetrain_->reset();
  forward_profile_.reset();
  rotation_profile_.reset();
  LOG_DEBUG("[Motion] Drive system reset complete.");
}

void Motion::stop() {
  LOG_DEBUG("[Motion] Stop called.");
  drivetrain_->stop();
  forward_profile_.reset();
  rotation_profile_.reset();
}

void Motion::disableDrive() {
  LOG_DEBUG("[Motion] Disable drive.");
  drivetrain_->stop();
}

// -----------------------------------------------------------------------------
// Forward motion
// -----------------------------------------------------------------------------
void Motion::startForward(float distance_mm, float top_speed, float final_speed,
                          float accel) {
  const float start_pos_mm = drivetrain_->getOdometry()->getDistanceMM();
  LOG_DEBUG("[Motion] Starting forward profile: Dist=" +
            std::to_string(distance_mm) + "mm, Vmax=" + std::to_string(top_speed) +
            "mm/s, Vend=" + std::to_string(final_speed) +
            "mm/s, Accel=" + std::to_string(accel) +
            "mm/s^2, StartPos=" + std::to_string(start_pos_mm));

  forward_profile_.start(distance_mm, top_speed, final_speed, accel,
                         start_pos_mm);
}

void Motion::forward(float distance_mm, float top_speed, float final_speed,
                     float accel, bool blocking) {
  float distanceMoved = 0.0f;

  LOG_DEBUG("[Motion] Forward() request: " + std::to_string(distance_mm) + " mm");
  startForward(distance_mm, top_speed, final_speed, accel);

  if (blocking) {
    while (!isForwardFinished()) {
      distanceMoved += drivetrain_->getOdometry()->getVelocityMMPerSec() * LOOP_INTERVAL_S;

      LOG_DEBUG("[Forward] Pos=" + std::to_string(positionMM()) +
                " mm, Speed=" + std::to_string(velocityMMPerSec()) + " mm/s");

      if (distanceMoved >= TOF_CELL_DEPTH_TO_CHECK_MM) {
        MulticoreSensorData local{};
        LOG_DEBUG("[Forward] Snapshot triggered at Pos=" +
                  std::to_string(positionMM()) + " mm");

        local.tof_left_exist = drivetrain_->isWallLeft();
        local.tof_front_exist = drivetrain_->isWallFront();
        local.tof_right_exist = drivetrain_->isWallRight();
        local.valid_sensors =
            (SensorMask)(static_cast<uint8_t>(SensorMask::TOF_LEFT_EXIST) |
                         static_cast<uint8_t>(SensorMask::TOF_FRONT_EXIST) |
                         static_cast<uint8_t>(SensorMask::TOF_RIGHT_EXIST));
        local.timestamp_ms = to_ms_since_boot(get_absolute_time());
        MulticoreSensorHub::publish(local);

        CommandHub::send(CommandType::SNAPSHOT,
                         static_cast<int32_t>(local.valid_sensors));
        distanceMoved = 0.0f;
      }

      update();
      sleep_ms(static_cast<int>(LOOP_INTERVAL_S * 1000));
    }
    LOG_DEBUG("[Forward] Finished forward motion.");
  }
}

bool Motion::isForwardFinished() const { return forward_profile_.isFinished(); }

// -----------------------------------------------------------------------------
// Rotational motion
// -----------------------------------------------------------------------------
void Motion::startTurn(float angle_deg, float omega, float final_omega,
                       float alpha) {
  const float current_deg = drivetrain_->getOdometry()->getAngleDeg();
  target_angle_deg_ = current_deg + angle_deg;

  LOG_DEBUG("[Motion] Starting turn profile: Angle=" + std::to_string(angle_deg) +
            " deg, Omega=" + std::to_string(omega) +
            " deg/s, Alpha=" + std::to_string(alpha) +
            " deg/s^2, StartAngle=" + std::to_string(current_deg));

  rotation_profile_.start(angle_deg, omega, final_omega, alpha, current_deg);
}

void Motion::turn(float angle_deg, float omega, float final_omega, float alpha,
                  bool blocking) {
  startTurn(angle_deg, omega, final_omega, alpha);
  LOG_DEBUG("[Turn] Requested turn of " + std::to_string(angle_deg) + " deg.");

  if (blocking) {
    while (!isTurnFinished()) {
      update();
      LOG_DEBUG("[Turn] Angle=" +
                std::to_string(drivetrain_->getOdometry()->getAngleDeg()) +
                " deg, Omega=" +
                std::to_string(drivetrain_->getOdometry()->getAngularVelocityDegPerSec()) +
                " deg/s");
      sleep_ms(static_cast<int>(LOOP_INTERVAL_S * 1000));
    }
    LOG_DEBUG("[Turn] Turn finished.");
  }
}

bool Motion::isTurnFinished() const {
  const float imu_angle = drivetrain_->getOdometry()->getAngleDeg();
  const float err = Wrap180(target_angle_deg_ - imu_angle);
  const float omega = drivetrain_->getOdometry()->getAngularVelocityDegPerSec();

  LOG_DEBUG("[TurnCheck] Target=" + std::to_string(target_angle_deg_) +
            " deg, Current=" + std::to_string(imu_angle) +
            " deg, Err=" + std::to_string(err) +
            " deg, Omega=" + std::to_string(omega) + " deg/s");

  return (std::fabs(err) < 1.0f) && (std::fabs(omega) < 2.0f);
}

// -----------------------------------------------------------------------------
// Integrated turns
// -----------------------------------------------------------------------------
void Motion::integratedTurn(float angle_deg, float omega, float alpha) {
  LOG_DEBUG("[IntegratedTurn] Angle=" + std::to_string(angle_deg) +
            " deg, Omega=" + std::to_string(omega) +
            " deg/s, Alpha=" + std::to_string(alpha));
  rotation_profile_.reset();
  const float start_angle = drivetrain_->getOdometry()->getAngleDeg();
  rotation_profile_.start(angle_deg, omega, 0.0f, alpha, start_angle);
}

void Motion::spinTurn(float angle_deg, float omega, float alpha) {
  LOG_DEBUG("[SpinTurn] Requested spin: " + std::to_string(angle_deg) +
            " deg at " + std::to_string(omega) + " deg/s");

  // 1. Decelerate forward motion to zero
  LOG_DEBUG("[SpinTurn] Decelerating forward profile to zero.");
  forward_profile_.start(0, velocityMMPerSec(), 0, accelerationMMPerSec2(),
                         positionMM());

  int safetyCount = 0;
  while (std::fabs(velocityMMPerSec()) > 1.0f) {
    update();
    LOG_DEBUG("[SpinTurn] Waiting stop... Vel=" +
              std::to_string(velocityMMPerSec()) + " mm/s");
    sleep_ms(static_cast<int>(LOOP_INTERVAL_S * 1000));
    if (++safetyCount > 200) {
      LOG_WARNING("[SpinTurn] Timeout waiting for forward stop!");
      break;
    }
  }

  LOG_DEBUG("[SpinTurn] Forward stopped. Starting rotation profile.");
  rotation_profile_.reset();
  float startAngle = drivetrain_->getOdometry()->getAngleDeg();
  rotation_profile_.start(angle_deg, omega, 0.0f, alpha, startAngle);
  LOG_DEBUG("[SpinTurn] Rotation profile started from " +
            std::to_string(startAngle) + " deg.");
}

// -----------------------------------------------------------------------------
// Stopping utilities
// -----------------------------------------------------------------------------
void Motion::stopAt(float target_mm) {
  const float current_pos = positionMM();
  const float remaining = target_mm - current_pos;
  LOG_DEBUG("[Motion] StopAt target=" + std::to_string(target_mm) +
            " mm (Remaining=" + std::to_string(remaining) + " mm)");
  forward_profile_.start(remaining, velocityMMPerSec(), 0.0f,
                         accelerationMMPerSec2(), current_pos);
}

void Motion::stopAfter(float distance_mm) {
  const float current_pos = positionMM();
  LOG_DEBUG("[Motion] StopAfter dist=" + std::to_string(distance_mm) +
            " mm from pos=" + std::to_string(current_pos));
  forward_profile_.start(distance_mm, velocityMMPerSec(), 0.0f,
                         accelerationMMPerSec2(), current_pos);
}

void Motion::waitUntilPosition(float target_mm) {
  LOG_DEBUG("[Motion] WaitUntilPosition: " + std::to_string(target_mm) + " mm");
  while (positionMM() < target_mm) {
    sleep_ms(2);
  }
}

void Motion::waitUntilDistance(float delta_mm) {
  waitUntilPosition(positionMM() + delta_mm);
}

// -----------------------------------------------------------------------------
// Update loop
// -----------------------------------------------------------------------------
void Motion::update() {
  const float current_mm = drivetrain_->getOdometry()->getDistanceMM();
  const float current_deg = drivetrain_->getOdometry()->getAngleDeg();
  const float current_omega = drivetrain_->getOdometry()->getAngularVelocityDegPerSec();

  forward_profile_.update(current_mm);
  rotation_profile_.update(current_deg);

  float fwdTarget = forward_profile_.speed();
  float rotTarget = rotation_profile_.speed();

  LOG_DEBUG("[Update] FwdTarget=" + std::to_string(fwdTarget) +
            " mm/s, RotTarget=" + std::to_string(rotTarget) +
            " deg/s, OdoAngle=" + std::to_string(current_deg) +
            " deg, OdoOmega=" + std::to_string(current_omega) + " deg/s");

  drivetrain_->runControl(fwdTarget, rotTarget, 0.0f);
}