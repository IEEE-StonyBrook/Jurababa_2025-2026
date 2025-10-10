/******************************************************************************
 * @file    Motion.cpp
 * @brief   Implementation of Motion class.
 ******************************************************************************/

#include "../../../Include/Platform/Pico/Robot/Motion.h"
#include "../../../Include/Platform/Pico/CommandHub.h"
#include "../../../Include/Platform/Pico/MulticoreSensors.h"


#include <cmath>
#include <string>

#include "../../../Include/Common/LogSystem.h"

namespace
{
/**
 * @brief Wrap an angle into [-180, 180] range for error calculation.
 */
inline float Wrap180(float angle_deg)
{
    if (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    if (angle_deg < -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}
} // namespace

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Motion::Motion(Drivetrain* drivetrain) : drivetrain_(drivetrain), target_angle_deg_(0.0f)
{
}

// -----------------------------------------------------------------------------
// System control
// -----------------------------------------------------------------------------
void Motion::resetDriveSystem()
{
    LOG_DEBUG("Resetting drive system...");
    drivetrain_->reset();
    forward_profile_.reset();
    rotation_profile_.reset();
    LOG_DEBUG("Drive system reset complete.");
}

void Motion::stop()
{
    drivetrain_->stop();
    forward_profile_.reset();
    rotation_profile_.reset();
}

void Motion::disableDrive()
{
    drivetrain_->stop();
}

// -----------------------------------------------------------------------------
// Forward motion
// -----------------------------------------------------------------------------
void Motion::startForward(float distance_mm, float top_speed, float final_speed, float accel)
{
    // Provide odometry start position to the profile so it can compute progress.
    const float start_pos_mm = drivetrain_->getOdometry()->getDistanceMM();
    forward_profile_.start(distance_mm, top_speed, final_speed, accel, start_pos_mm);
}

void Motion::forward(float distance_mm, float top_speed, float final_speed, float accel,
                     bool blocking)
{
    float distanceMoved = 0.0f;

    LOG_DEBUG("Starting forward motion: " + std::to_string(distance_mm) + " mm");
    startForward(distance_mm, top_speed, final_speed, accel);

    if (blocking)
    {
        while (!isForwardFinished())
        {
            distanceMoved =
                drivetrain_->getOdometry()->getVelocityMMPerSec() * LOOP_INTERVAL_S + distanceMoved;
            // LOG_DEBUG("IsForwardFinished: " + std::to_string(isForwardFinished()));
            LOG_DEBUG("Pos: " + std::to_string(positionMM()) +
                      " mm, Speed: " + std::to_string(velocityMMPerSec()) + " mm/s");
            // LOG_DEBUG("Angle: " + std::to_string(angleDeg()) +
            //           " deg, Omega: " + std::to_string(omegaDegPerSec()) + "
            //           deg/s");
            if (distanceMoved == TOF_CELL_DEPTH_TO_CHECK_MM)
            {
                MulticoreSensorData local{};
                LOG_DEBUG("Publishing sensor snapshot at " + std::to_string(positionMM()) + " mm");

                local.tof_left_exist  = drivetrain_->isWallLeft();
                local.tof_front_exist = drivetrain_->isWallFront();
                local.tof_right_exist = drivetrain_->isWallRight();
                local.valid_sensors =
                    (SensorMask)(static_cast<uint8_t>(SensorMask::TOF_LEFT_EXIST) |
                                 static_cast<uint8_t>(SensorMask::TOF_FRONT_EXIST) |
                                 static_cast<uint8_t>(SensorMask::TOF_RIGHT_EXIST));
                local.timestamp_ms = to_ms_since_boot(get_absolute_time());
                MulticoreSensorHub::publish(local);

                CommandHub::send(CommandType::SNAPSHOT, static_cast<int32_t>(local.valid_sensors));
                distanceMoved = 0.0f;
            }
            update();
            sleep_ms(static_cast<int>(LOOP_INTERVAL_S * 1000));
        }
    }
}

bool Motion::isForwardFinished() const
{
    return forward_profile_.isFinished();
}

// -----------------------------------------------------------------------------
// Rotational motion
// -----------------------------------------------------------------------------
void Motion::startTurn(float angle_deg, float omega, float final_omega, float alpha)
{
    // Record absolute IMU-based target angle (for truth-based completion).
    const float current_deg = drivetrain_->getOdometry()->getAngleDeg();
    target_angle_deg_       = current_deg + angle_deg;

    // Provide IMU start angle to the profile so it can compute progress.
    rotation_profile_.start(angle_deg, omega, final_omega, alpha, current_deg);
}

void Motion::turn(float angle_deg, float omega, float final_omega, float alpha, bool blocking)
{
    startTurn(angle_deg, omega, final_omega, alpha);
    LOG_DEBUG("Starting turn: " + std::to_string(angle_deg) + " degrees");

    if (blocking)
    {
        while (!isTurnFinished())
        {
            update();
            LOG_DEBUG("Angle: " + std::to_string(drivetrain_->getOdometry()->getAngleDeg()) +
                      " deg, Omega: " +
                      std::to_string(drivetrain_->getOdometry()->getAngularVelocityDegPerSec()) +
                      " deg/s");
            sleep_ms(static_cast<int>(LOOP_INTERVAL_S * 1000));
        }
    }
}

bool Motion::isTurnFinished() const
{
    const float imu_angle = drivetrain_->getOdometry()->getAngleDeg();
    const float err       = Wrap180(target_angle_deg_ - imu_angle);
    return std::fabs(err) < 0.1f; // 0.1 degree tolerance
}

// -----------------------------------------------------------------------------
// Integrated turns
// -----------------------------------------------------------------------------
void Motion::integratedTurn(float angle_deg, float omega, float alpha)
{
    rotation_profile_.reset();
    const float start_angle = drivetrain_->getOdometry()->getAngleDeg();
    rotation_profile_.start(angle_deg, omega, 0.0f, alpha, start_angle);
}

void Motion::spinTurn(float angle_deg, float omega, float alpha)
{
    drivetrain_->runControl(0.0f, 0.0f, 0.0f);
    integratedTurn(angle_deg, omega, alpha);
}

// -----------------------------------------------------------------------------
// Stopping utilities
// -----------------------------------------------------------------------------
void Motion::stopAt(float target_mm)
{
    // Build a decel profile to the specific position target.
    const float current_pos = positionMM();
    const float remaining   = target_mm - current_pos;
    forward_profile_.start(remaining,
                           velocityMMPerSec(), // use current speed as max to ramp down
                           0.0f,               // final stop
                           accelerationMMPerSec2(),
                           current_pos); // start position (odometry)
}

void Motion::stopAfter(float distance_mm)
{
    // Build a decel profile beginning now for a relative distance.
    const float current_pos = positionMM();
    forward_profile_.start(distance_mm, velocityMMPerSec(), 0.0f, accelerationMMPerSec2(),
                           current_pos); // start position (odometry)
}

void Motion::waitUntilPosition(float target_mm)
{
    while (positionMM() < target_mm)
    {
        sleep_ms(2);
    }
}

void Motion::waitUntilDistance(float delta_mm)
{
    waitUntilPosition(positionMM() + delta_mm);
}

// -----------------------------------------------------------------------------
// Update loop
// -----------------------------------------------------------------------------
void Motion::update()
{
    // Feed CURRENT odometry positions to the profiles so they can compute
    // remaining.
    const float current_mm  = drivetrain_->getOdometry()->getDistanceMM();
    const float current_deg = drivetrain_->getOdometry()->getAngleDeg();

    forward_profile_.update(current_mm);
    rotation_profile_.update(current_deg);

    // Drivetrain expects forward (mm/s) and rotation (deg/s).
    drivetrain_->runControl(forward_profile_.speed(), rotation_profile_.speed(), 0.0f);
}