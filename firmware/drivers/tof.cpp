#include "drivers/tof.h"

#include <ctype.h>

#include "config/config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

ToF::ToF(int xshut_pin, char sensor_position) : sensor_position_(sensor_position)
{
    setup_xshut_pin(xshut_pin);
    reset_sensor(xshut_pin);
    initialize_sensor(xshut_pin, sensor_position);
    setup_continuous_ranging();
}

void ToF::setup_xshut_pin(int xshut_pin)
{
    gpio_init(xshut_pin);
    gpio_set_dir(xshut_pin, GPIO_OUT);
    sleep_ms(10);
}

void ToF::reset_sensor(int xshut_pin)
{
    gpio_put(xshut_pin, 0);
}

void ToF::initialize_sensor(int xshut_pin, char sensor_position)
{
    gpio_put(xshut_pin, 1);
    sleep_ms(10);

    sensor_device_.I2cDevAddr      = 0x29;
    sensor_device_.comms_type      = 1;
    sensor_device_.comms_speed_khz = 400;

    VL53L0X_Error status =
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

    status                    = VL53L0X_SetDeviceAddress(&sensor_device_, new_address);
    sensor_device_.I2cDevAddr = new_address;
}

void ToF::setup_continuous_ranging()
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

float ToF::get_distance()
{
    VL53L0X_RangingMeasurementData_t measurement_data;
    VL53L0X_GetRangingMeasurementData(&sensor_device_, &measurement_data);
    VL53L0X_ClearInterruptMask(&sensor_device_, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    // RangeStatus == 0 means a valid measurement. On error we leave the
    // cache untouched, so callers see the last good reading (or the 8191
    // out-of-range sentinel if no valid read has happened yet).
    if (measurement_data.RangeStatus == 0)
    {
        last_valid_distance_ = measurement_data.RangeMilliMeter;
    }
    return last_valid_distance_;
}
