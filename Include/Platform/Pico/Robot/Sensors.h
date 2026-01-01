#ifndef SENSORS_H
#define SENSORS_H

#include "Platform/Pico/Robot/IMU.h"
#include "Platform/Pico/Robot/ToF.h"

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

    // Returns current angular velocity in degrees per second
    float getAngularVelocityDegps();

    // Resets current yaw to zero
    void resetYaw();

    // Update sensor readings; call every control tick with dt in seconds
    void update(float dt);

  private:
    // Inertial measurement unit (Heading)
    IMU* imu;

    // Time-of-flight distance sensors
    ToF* leftToF;
    ToF* frontToF;
    ToF* rightToF;

    // For angular velocity calculations
    float lastYaw           = 0.0f;
    float currentAngularVel = 0.0f;
};

#endif