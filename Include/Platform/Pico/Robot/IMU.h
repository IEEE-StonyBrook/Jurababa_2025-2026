#ifndef IMU_H
#define IMU_H

#include <array>

#include "pico/stdlib.h"

class IMU {
 public:
  IMU(int uartRXPin = 5);

  float getIMUYawDegreesNeg180ToPos180();
  void resetIMUYawToZero();
  float getNewYawAfterAddingDegrees(float degreesToAdd);

 private:
  void setUpIMUCommunication();
  void setUpIMUInterrupts();
  void processIMURXInterruptData();
  void convertPacketDataToUsableYaw();
  static void imuInterruptHandler();
  static IMU* imuInstance;


  const int uartRXPin;
  volatile uint8_t IMUBufferForYaw[19];
  float robotYawNeg180To180Degrees;
  float resetOffSet;
};

#endif