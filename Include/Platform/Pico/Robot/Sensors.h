#ifndef SENSORS_H
#define SENSORS_H

#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/Config.h"
#include "IMU.h"
#include "ToF.h"

class Sensors
{
  public:
    Sensors(IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF);

    // Reset odometry and all controller states.
    void reset();

    // Drive straight forward a set distance in mm at given velocity.
    void driveForwardMM(float distanceMM, float velocityMMPerSec = 300.0f);

    // ToF sensor wall detection.
    bool isWallLeft();
    bool isWallFront();
    bool isWallRight();

  private:
    ToF* leftToF;
    ToF* frontToF;
    ToF* rightToF;
    IMU* imu;
};

#endif