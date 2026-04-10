#include "drivers/line_sensor.h"

#include "common/log.h"

LineSensor::LineSensor(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t addr)
    : i2c_(i2c), sda_pin_(sda_pin), scl_pin_(scl_pin), addr_(addr)
{
}

void LineSensor::init()
{
    i2c_init(i2c_, LINE_SENSOR_I2C_BAUD);

    gpio_set_function(sda_pin_, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin_);
    gpio_pull_up(scl_pin_);

    // Initialization sequence from vendor sample code:
    // Write 1 to register 0x01, wait, then write 0
    uint8_t init_on[]  = {0x01, 0x01};
    uint8_t init_off[] = {0x01, 0x00};

    i2c_write_blocking(i2c_, addr_, init_on, 2, false);
    sleep_ms(100);
    i2c_write_blocking(i2c_, addr_, init_off, 2, false);
    sleep_ms(100);

    LOG_INFO("LineSensor: I2C initialized on SDA=" + std::to_string(sda_pin_) +
             " SCL=" + std::to_string(scl_pin_) + " addr=0x" + std::to_string(addr_));
}

void LineSensor::read()
{
    // Read from sensor data register
    uint8_t reg = LINE_SENSOR_DATA_REG;
    uint8_t buf = 0;

    // Write register address, then read data
    int ret = i2c_write_blocking(i2c_, addr_, &reg, 1, true); // keep bus active
    if (ret == PICO_ERROR_GENERIC)
    {
        LOG_DEBUG("LineSensor: I2C write reg failed");
        return;
    }

    ret = i2c_read_blocking(i2c_, addr_, &buf, 1, false);
    if (ret == PICO_ERROR_GENERIC)
    {
        LOG_DEBUG("LineSensor: I2C read failed");
        return;
    }

    sensor_data_ = buf;
}

uint8_t LineSensor::getSensorData() const
{
    return sensor_data_;
}

float LineSensor::getPosition() const
{
    // Weighted average: sensor positions from -3.5 (leftmost) to +3.5 (rightmost)
    // Bit 0 = leftmost = position -3.5
    // Bit 7 = rightmost = position +3.5
    float weighted_sum = 0.0f;
    float active_count = 0.0f;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (sensor_data_ & (1 << i))
        {
            float position = static_cast<float>(i) - 3.5f;
            weighted_sum += position;
            active_count += 1.0f;
        }
    }

    if (active_count < 0.5f)
    {
        return 0.0f; // No sensors active, return centered
    }

    return weighted_sum / active_count;
}

bool LineSensor::onLine() const
{
    return sensor_data_ != 0;
}

bool LineSensor::detectIntersection() const
{
    // Check if 2 leftmost sensors (bits 0 and 1) are both active
    bool left_intersection = (sensor_data_ & 0x03) == 0x03;

    // Check if 2 rightmost sensors (bits 6 and 7) are both active
    bool right_intersection = (sensor_data_ & 0xC0) == 0xC0;

    return left_intersection || right_intersection;
}
