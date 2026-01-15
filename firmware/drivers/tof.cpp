#include "drivers/tof.h"

#include <ctype.h>

#include "config/config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#ifdef USE_MULTICORE_SENSORS
#include "app/multicore.h"
#endif

ToF::ToF(int xshut_pin, char sensor_position) : sensor_position_(sensor_position)
{
    setupXSHUTPin(xshut_pin);
    resetSensor(xshut_pin);
    initializeSensor(xshut_pin, sensor_position);
    setupContinuousRanging();
}

void ToF::setupXSHUTPin(int xshut_pin)
{
    gpio_init(xshut_pin);
    gpio_set_dir(xshut_pin, GPIO_OUT);
    sleep_ms(10);
}

void ToF::resetSensor(int xshut_pin)
{
    gpio_put(xshut_pin, 0);
}

void ToF::initializeSensor(int xshut_pin, char sensor_position)
{
    gpio_put(xshut_pin, 1);
    sleep_ms(10);

    sensor_device_.I2cDevAddr      = 0x29;
    sensor_device_.comms_type      = 1;
    sensor_device_.comms_speed_khz = 400;

    VL53L0X_dev_i2c_default_initialise(&sensor_device_, VL53L0X_DEFAULT_MODE);

    uint8_t new_address;
    switch (tolower(sensor_position))
    {
        case 'l':
            new_address = 0x30;
            break;
        case 'f':
            new_address = 0x31;
            break;
        case 'r':
            new_address = 0x32;
            break;
        default:
            new_address = 0x33;
            break;
    }

    VL53L0X_SetDeviceAddress(&sensor_device_, new_address);
    sensor_device_.I2cDevAddr = new_address;
}

void ToF::setupContinuousRanging()
{
    VL53L0X_WaitDeviceBooted(&sensor_device_);
    VL53L0X_DataInit(&sensor_device_);
    VL53L0X_StaticInit(&sensor_device_);
    VL53L0X_PerformRefCalibration(&sensor_device_, nullptr, nullptr);
    VL53L0X_SetDeviceMode(&sensor_device_, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&sensor_device_, TOF_TIMING_BUDGET_US);
    VL53L0X_SetInterMeasurementPeriodMilliSeconds(&sensor_device_, TOF_MEASUREMENT_PERIOD_MS);
    VL53L0X_StartMeasurement(&sensor_device_);
}

float ToF::distance()
{
    VL53L0X_RangingMeasurementData_t measurement_data;
    VL53L0X_GetRangingMeasurementData(&sensor_device_, &measurement_data);
    VL53L0X_ClearInterruptMask(&sensor_device_, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    float distance_mm = measurement_data.RangeMilliMeter;

#ifdef USE_MULTICORE_SENSORS
    char position_lower = tolower(sensor_position_);

    if (multicore_get_core_num() == 1)
    {
        MulticoreSensorData sensor_data = {};

        switch (position_lower)
        {
            case 'l':
                sensor_data.tof_left_mm = static_cast<int16_t>(distance_mm);
                break;
            case 'r':
                sensor_data.tof_right_mm = static_cast<int16_t>(distance_mm);
                break;
            default:
                sensor_data.tof_front_mm = static_cast<int16_t>(distance_mm);
                break;
        }

        sensor_data.timestamp_ms = to_ms_since_boot(get_absolute_time());
        MulticoreSensorHub::publish(sensor_data);

        return distance_mm;
    }

    MulticoreSensorData snapshot = {};
    MulticoreSensorHub::snapshot(snapshot);

    switch (position_lower)
    {
        case 'l':
            return static_cast<float>(snapshot.tof_left_mm);
        case 'r':
            return static_cast<float>(snapshot.tof_right_mm);
        default:
            return static_cast<float>(snapshot.tof_front_mm);
    }
#else
    return distance_mm;
#endif
}
