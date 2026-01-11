#include "Platform/Pico/Robot/Sensors.h"

#include "Platform/Pico/Config.h"

namespace
{
    /**
     * @brief Normalizes yaw delta to [-180, 180] range
     *
     * Handles wrap-around: turning from +179 to -179 should be +2, not -358.
     */
    float normalizeYawDelta(float delta)
    {
        if (delta > 180.0f)
            return delta - 360.0f;
        if (delta < -180.0f)
            return delta + 360.0f;
        return delta;
    }
}  // namespace

Sensors::Sensors(IMU* imu, ToF* left_tof, ToF* front_tof, ToF* right_tof)
    : imu_(imu),
      left_tof_(left_tof),
      front_tof_(front_tof),
      right_tof_(right_tof)
{
}

bool Sensors::isWallLeft()
{
    return left_tof_->getToFDistanceFromWallMM() < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Sensors::isWallFront()
{
    return front_tof_->getToFDistanceFromWallMM() < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool Sensors::isWallRight()
{
    return right_tof_->getToFDistanceFromWallMM() < TOF_RIGHT_WALL_THRESHOLD_MM;
}

float Sensors::getYaw()
{
    return imu_->getIMUYawDegreesNeg180ToPos180();
}

float Sensors::getAngularVelocityDegps()
{
    return current_angular_velocity_deg_per_second_;
}

float Sensors::getYawDelta()
{
    float current_yaw = getYaw();
    float delta = normalizeYawDelta(current_yaw - last_yaw_for_delta_);
    last_yaw_for_delta_ = current_yaw;
    return delta;
}

float Sensors::getFrontDistanceMM()
{
    return front_tof_->getToFDistanceFromWallMM();
}

float Sensors::getLeftDistanceMM()
{
    return left_tof_->getToFDistanceFromWallMM();
}

float Sensors::getRightDistanceMM()
{
    return right_tof_->getToFDistanceFromWallMM();
}

void Sensors::resetYaw()
{
    imu_->resetIMUYawToZero();
    last_yaw_for_delta_ = 0.0f;  // Reset delta tracking
}

void Sensors::resetPosition()
{
    // TODO: This should coordinate with Drivetrain to reset encoder positions
    // For now, just reset yaw delta tracking
    last_yaw_for_delta_ = getYaw();
}

void Sensors::update(float time_delta)
{
    if (time_delta < 0.0001f)
        return;

    float current_yaw = getYaw();
    float delta_yaw = normalizeYawDelta(current_yaw - previous_yaw_);
    float raw_angular_velocity = delta_yaw / time_delta;

    // Low-pass filter to reduce noise
    float alpha = SENSORS_ANGULAR_VEL_FILTER_ALPHA;
    current_angular_velocity_deg_per_second_ =
        alpha * raw_angular_velocity +
        (1.0f - alpha) * current_angular_velocity_deg_per_second_;

    previous_yaw_ = current_yaw;
}