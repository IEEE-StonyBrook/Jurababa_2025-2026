#include "drivers/line_sensor.h"

#include "common/log.h"

LineSensor::LineSensor(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t addr)
    : i2c_(i2c), sda_pin_(sda_pin), scl_pin_(scl_pin), addr_(addr)
{
}

void LineSensor::begin()
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

bool LineSensor::read()
{
    // Read from sensor data register
    uint8_t reg = LINE_SENSOR_DATA_REG;
    uint8_t buf = 0;

    // Write register address, then read data
    int ret = i2c_write_blocking(i2c_, addr_, &reg, 1, true); // keep bus active
    if (ret == PICO_ERROR_GENERIC)
    {
        LOG_DEBUG("LineSensor: I2C write reg failed");
        read_valid_ = false;
        return false;
    }

    ret = i2c_read_blocking(i2c_, addr_, &buf, 1, false);
    if (ret == PICO_ERROR_GENERIC)
    {
        LOG_DEBUG("LineSensor: I2C read failed");
        read_valid_ = false;
        return false;
    }

    raw_data_   = buf;
    read_valid_ = true;
    updateDerivedState();
    return true;
}

void LineSensor::updateDerivedState()
{
    active_mask_ = LINE_SENSOR_ACTIVE_LOW ? static_cast<uint8_t>(~raw_data_) : raw_data_;

    // Weighted average: bit 0 / X8 is leftmost (-3.5), bit 7 / X1 is
    // rightmost (+3.5). This is the mounted orientation measured with LINCON.
    float weighted_sum = 0.0f;
    float active_count = 0.0f;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (active_mask_ & (1 << i))
        {
            float position = static_cast<float>(i) - 3.5f;
            weighted_sum += position;
            active_count += 1.0f;
        }
    }

    if (active_count < 0.5f)
    {
        position_valid_ = false;
        position_       = last_position_;
        return;
    }

    position_valid_ = true;
    position_       = weighted_sum / active_count;
    last_position_  = position_;
}

float LineSensor::get_position() const
{
    return position_;
}

bool LineSensor::on_line() const
{
    return position_valid_;
}

bool LineSensor::detect_intersection()
{
    const bool left_edge_active  = (active_mask_ & 0x03) == 0x03;
    const bool right_edge_active = (active_mask_ & 0xC0) == 0xC0;
    const uint32_t now_ms        = to_ms_since_boot(get_absolute_time());

    // UKMARS-style transition logic: don't require both sides to be active in
    // the same instant. Angled crossings often light one side first.
    if (!intersection_pending_)
    {
        if (!left_edge_active && !right_edge_active)
            return false;

        intersection_pending_         = true;
        intersection_window_start_ms_ = now_ms;
        intersection_left_seen_       = left_edge_active;
        intersection_right_seen_      = right_edge_active;
    }
    else
    {
        intersection_left_seen_  = intersection_left_seen_ || left_edge_active;
        intersection_right_seen_ = intersection_right_seen_ || right_edge_active;
    }

    if (intersection_left_seen_ && intersection_right_seen_)
    {
        intersection_pending_    = false;
        intersection_left_seen_  = false;
        intersection_right_seen_ = false;
        return true;
    }

    if ((now_ms - intersection_window_start_ms_) > LINE_INTERSECTION_WINDOW_MS)
    {
        if (left_edge_active || right_edge_active)
        {
            // Keep tracking when still on a complex marker region.
            intersection_window_start_ms_ = now_ms;
            intersection_left_seen_       = left_edge_active;
            intersection_right_seen_      = right_edge_active;
            intersection_pending_         = true;
        }
        else
        {
            intersection_pending_    = false;
            intersection_left_seen_  = false;
            intersection_right_seen_ = false;
        }
    }

    return false;
}

uint8_t LineSensor::rawByte() const
{
    return raw_data_;
}

uint8_t LineSensor::activeMask() const
{
    return active_mask_;
}
