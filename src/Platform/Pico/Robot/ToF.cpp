#include "../../../Include/Platform/Pico/Robot/ToF.h"

#include <cctype>

#include "hardware/gpio.h"

// Left (l), Front (F), Right (R)
ToF::ToF(int xShutToFPin, char ToFPosition) {
  setUpToFPin(xShutToFPin);
  resetToFToZeroDegrees(xShutToFPin);
  setUpToFSensor(xShutToFPin, ToFPosition);
  setUpToFContinuousSensing();
}

void ToF::setUpToFPin(int xShutToFPin) {
  gpio_init(xShutToFPin);
  gpio_set_dir(xShutToFPin, GPIO_OUT);
  sleep_ms(10);
}

void ToF::resetToFToZeroDegrees(int xShutToFPin) { gpio_put(xShutToFPin, 0); }

void ToF::setUpToFSensor(int xShutToFPin, char ToFPosition) {
  gpio_put(xShutToFPin, 1);
  sleep_ms(10);
  ToFSensor.I2cDevAddr = 0x29;
  ToFSensor.comms_type = 1;
  ToFSensor.comms_speed_khz = 400;
  VL53L0X_dev_i2c_default_initialise(&ToFSensor, VL53L0X_DEFAULT_MODE);

  uint8_t ToFI2CAddress;
  if (tolower(ToFPosition) == 'l')
    ToFI2CAddress = 0x30;
  else if (tolower(ToFPosition) == 'f')
    ToFI2CAddress = 0x31;
  else if (tolower(ToFPosition) == 'r')
    ToFI2CAddress = 0x32;
  else
    ToFI2CAddress = 0x33;
  bool successfulSensorInstantiation =
      VL53L0X_SetDeviceAddress(&ToFSensor, ToFI2CAddress);
  ToFSensor.I2cDevAddr = ToFI2CAddress;
}

void ToF::setUpToFContinuousSensing() {
  VL53L0X_WaitDeviceBooted(&ToFSensor);
  VL53L0X_DataInit(&ToFSensor);
  VL53L0X_StaticInit(&ToFSensor);
  VL53L0X_PerformRefCalibration(&ToFSensor, nullptr, nullptr);

  VL53L0X_SetDeviceMode(&ToFSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&ToFSensor, 20000);
  VL53L0X_SetInterMeasurementPeriodMilliSeconds(&ToFSensor, 30);
  VL53L0X_StartMeasurement(&ToFSensor);
}

float ToF::getToFDistanceFromWallMM() {
  VL53L0X_RangingMeasurementData_t distanceFromWall;

  VL53L0X_GetRangingMeasurementData(&ToFSensor, &distanceFromWall);
  VL53L0X_ClearInterruptMask(
      &ToFSensor, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

  return distanceFromWall.RangeMilliMeter;
}