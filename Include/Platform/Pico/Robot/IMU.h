#ifndef IMU_H
#define IMU_H

#include <cmath>
#include <cstdint>

#include "../../Include/Common/LogSystem.h"
#include "../Config.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

class IMU
{
  public:
    explicit IMU(int uartRXPin);

    // Get yaw in degrees normalized to [-180, 180].
    float getIMUYawDegreesNeg180ToPos180();

    // Reset yaw offset to make current yaw = 0.
    void resetIMUYawToZero();

    // Get yaw after adding a delta angle, normalized to [-180, 180].
    float getNewYawAfterAddingDegrees(float degreesToAdd);

  private:
    const int    uartRXPin;                       // RX pin used by UART.
    volatile int IMUBufferIndex;                  // Index for received bytes.
    uint8_t      IMUBufferForYaw[IMU_PACKET_LEN]; // Packet buffer.
    bool         yawReady;                        // Whether a valid yaw has been read.

    float robotYawNeg180To180Degrees; // Latest yaw reading.
    float resetOffSet;                // Zero offset for yaw.

    static IMU* imuInstance; // Static instance pointer.

    // UART + Interrupt setup.
    void        setUpIMUCommunication();
    void        setUpIMUInterrupts();
    void        processIMURXInterruptData();
    void        convertPacketDataToUsableYaw();
    static void imuInterruptHandler();
};
#endif