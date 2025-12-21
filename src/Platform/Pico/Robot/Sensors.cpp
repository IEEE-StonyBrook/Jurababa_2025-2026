#include "../../../Include/Platform/Pico/Robot/Sensors.h"

Sensors::Sensors(IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF)
    : imu(imu), leftToF(leftToF), frontToF(frontToF), rightToF(rightToF)
{
}

void Sensors::reset()
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