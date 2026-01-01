#include "Platform/Pico/Robot/Sensors.h"

Sensors::Sensors(IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF)
    : imu(imu), leftToF(leftToF), frontToF(frontToF), rightToF(rightToF)
{
}

bool Sensors::isWallLeft()
{
    return leftToF->getToFDistanceFromWallMM() < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Sensors::isWallFront()
{
    return frontToF->getToFDistanceFromWallMM() < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool Sensors::isWallRight()
{
    return rightToF->getToFDistanceFromWallMM() < TOF_RIGHT_WALL_THRESHOLD_MM;
}

float Sensors::getYaw()
{
    return imu->getIMUYawDegreesNeg180ToPos180();
}

float Sensors::getAngularVelocityDegps()
{
    return currentAngularVel;
}

void Sensors::resetYaw()
{
    imu->resetIMUYawToZero();
}

void Sensors::update(float dt)
{
    if (dt < 0.0001f)
        return;

    float currentYaw = getYaw();

    // 1. Calculate the change in yaw
    float deltaYaw = currentYaw - lastYaw;

    // 2. IMPORTANT: Handle the -180 to 180 wrap-around
    // If we turn from 179 to -179, deltaYaw is -358, but it should be 2.
    if (deltaYaw > 180.0f)
        deltaYaw -= 360.0f;
    if (deltaYaw < -180.0f)
        deltaYaw += 360.0f;

    // 3. Calculate velocity (degrees per second)
    float rawVel = deltaYaw / dt;

    // 4. Optional: Simple Low-Pass Filter to smooth out noise
    // alpha = 1.0 means no filtering, 0.1 means heavy filtering
    float alpha       = 0.7f;
    currentAngularVel = (alpha * rawVel) + (1.0f - alpha) * currentAngularVel;

    lastYaw = currentYaw;
}