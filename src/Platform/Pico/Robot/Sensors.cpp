#include "Platform/Pico/Robot/Sensors.h"
#include "Platform/Pico/Config.h"

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

void Sensors::resetYaw()
{
    imu_->resetIMUYawToZero();
}

void Sensors::update(float time_delta)
{
    if (time_delta < 0.0001f)
        return;

    float current_yaw = getYaw();
    float delta_yaw = current_yaw - previous_yaw_;

    // Handle wrap-around: turning from +179° to -179° should be +2°, not -358°
    if (delta_yaw > 180.0f)
        delta_yaw -= 360.0f;
    else if (delta_yaw < -180.0f)
        delta_yaw += 360.0f;

    float raw_angular_velocity = delta_yaw / time_delta;

    // Low-pass filter to reduce noise
    float alpha = SENSORS_ANGULAR_VEL_FILTER_ALPHA;
    current_angular_velocity_deg_per_second_ =
        (alpha * raw_angular_velocity) +
        (1.0f - alpha) * current_angular_velocity_deg_per_second_;

    previous_yaw_ = current_yaw;
}