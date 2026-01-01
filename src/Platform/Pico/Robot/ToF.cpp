#include "Platform/Pico/Robot/ToF.h"

#include <cctype>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#ifdef USE_MULTICORE_SENSORS
#include "Platform/Pico/MulticoreSensors.h"
#endif

// Left (l), Front (F), Right (R)
ToF::ToF(int xShutToFPin, char ToFPosition)
{
    setUpToFPin(xShutToFPin);
    resetToFToZeroDegrees(xShutToFPin);
    setUpToFSensor(xShutToFPin, ToFPosition);
    this->ToFPosition = ToFPosition;
    setUpToFContinuousSensing();
}

void ToF::setUpToFPin(int xShutToFPin)
{
    gpio_init(xShutToFPin);
    gpio_set_dir(xShutToFPin, GPIO_OUT);
    sleep_ms(10);
}

void ToF::resetToFToZeroDegrees(int xShutToFPin)
{
    gpio_put(xShutToFPin, 0);
}

void ToF::setUpToFSensor(int xShutToFPin, char ToFPosition)
{
    gpio_put(xShutToFPin, 1);
    sleep_ms(10);
    ToFSensor.I2cDevAddr      = 0x29;
    ToFSensor.comms_type      = 1;
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
    bool successfulSensorInstantiation = VL53L0X_SetDeviceAddress(&ToFSensor, ToFI2CAddress);
    ToFSensor.I2cDevAddr               = ToFI2CAddress;
}

void ToF::setUpToFContinuousSensing()
{
    VL53L0X_WaitDeviceBooted(&ToFSensor);
    VL53L0X_DataInit(&ToFSensor);
    VL53L0X_StaticInit(&ToFSensor);
    VL53L0X_PerformRefCalibration(&ToFSensor, nullptr, nullptr);

    VL53L0X_SetDeviceMode(&ToFSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&ToFSensor, 20000);
    VL53L0X_SetInterMeasurementPeriodMilliSeconds(&ToFSensor, 30);
    VL53L0X_StartMeasurement(&ToFSensor);
}

float ToF::getToFDistanceFromWallMM()
{
    VL53L0X_RangingMeasurementData_t distanceFromWall;

    VL53L0X_GetRangingMeasurementData(&ToFSensor, &distanceFromWall);
    VL53L0X_ClearInterruptMask(&ToFSensor, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    float mm = distanceFromWall.RangeMilliMeter;
    // If multicore is enabled and this is a consumer core, return the latest
    // snapshot from the hub instead of publishing from here. If this code is
    // running on the publisher core (core 1) we still publish so the hub has
    // fresh data.
#ifdef USE_MULTICORE_SENSORS
    if (multicore_get_core_num() == 1)
    {
        MulticoreSensorData s = {};
        if (tolower(ToFPosition) == 'l')
            s.tof_left_mm = (int16_t)mm;
        else if (tolower(ToFPosition) == 'f')
            s.tof_front_mm = (int16_t)mm;
        else if (tolower(ToFPosition) == 'r')
            s.tof_right_mm = (int16_t)mm;
        else
            s.tof_front_mm = (int16_t)mm;

        s.timestamp_ms = to_ms_since_boot(get_absolute_time());
        MulticoreSensorHub::publish(s);

        return mm;
    }

    MulticoreSensorData s = {};
    MulticoreSensorHub::snapshot(s);
    if (tolower(ToFPosition) == 'l')
        return (float)s.tof_left_mm;
    else if (tolower(ToFPosition) == 'f')
        return (float)s.tof_front_mm;
    else if (tolower(ToFPosition) == 'r')
        return (float)s.tof_right_mm;
    else
        return (float)s.tof_front_mm;
#else
    return mm;
#endif
}