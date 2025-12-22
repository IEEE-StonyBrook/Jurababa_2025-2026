#ifndef SENSORS_H
#define SENSORS_H

#include "../../../Include/Platform/Pico/Robot/IMU.h"
#include "../../../Include/Platform/Pico/Robot/ToF.h"

class Sensors
{
  public:
    // Construct sensors wrapper with IMU and ToF sensors
    Sensors(IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF);

    // Returns true if a wall is detected on the left
    bool isWallLeft();

    // Returns true if a wall is detected in front
    bool isWallFront();

    // Returns true if a wall is detected on the right
    bool isWallRight();

    // Returns current yaw angle in degrees
    // Range is [-180, 180]
    float getYaw();

    // Resets current yaw to zero
    void resetYaw();

  private:
    // Inertial measurement unit (Heading)
    IMU* imu;

    // Time-of-flight distance sensors
    ToF* leftToF;
    ToF* frontToF;
    ToF* rightToF;
};

#endif