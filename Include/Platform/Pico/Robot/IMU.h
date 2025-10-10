/******************************************************************************
 * @file    IMU.h
 * @brief   Minimal I2C-based IMU driver for BNO085 (yaw only)
 ******************************************************************************/

#ifndef IMU_H
#define IMU_H

#include <cstdint>

class IMU {
public:
    IMU();

    /**
     * @brief Enable rotation vector feature on the IMU (must be called once after init).
     * @return true if the request was successfully sent over I2C
     */
    bool enableRotationVector();

    /**
     * @brief Get the current yaw angle in degrees [-180, 180],
     *        corrected for any reset offset.
     * @return yaw in degrees (float)
     */
    float getIMUYawDegreesNeg180ToPos180();

    /**
     * @brief Reset yaw so the current heading becomes 0°.
     */
    void resetIMUYawToZero();

private:
    static IMU* imuInstance;

    // Latest raw yaw value from IMU
    float robotYawDegrees;

    // Offset applied when resetting yaw
    float yawOffset;

    // Tracks if IMU has produced valid data yet
    bool yawReady;

    // ---- Internal Helpers ----
    bool sendPacket(uint8_t channel, const uint8_t* data, uint16_t len);
    bool readPacket(uint8_t &channel, uint16_t &len);
    void updateYaw();
};

#endif // IMU_H