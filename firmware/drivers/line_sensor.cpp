#include "drivers/line_sensor.h"

#include "common/log.h"

namespace
{
bool maskActive(uint8_t active_mask, uint8_t required_mask)
{
    return (active_mask & required_mask) == required_mask;
}
} // namespace

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

uint8_t LineSensor::classifyPaths(uint8_t active_mask) const
{
    uint8_t paths = PATH_NONE;
    if (maskActive(active_mask, LINE_INTERSECTION_LEFT_MASK))
        paths |= PATH_LEFT;
    if (maskActive(active_mask, LINE_INTERSECTION_FORWARD_MASK))
        paths |= PATH_FORWARD;
    if (maskActive(active_mask, LINE_INTERSECTION_RIGHT_MASK))
        paths |= PATH_RIGHT;
    return paths;
}

uint8_t LineSensor::currentPathsMask() const
{
    return classifyPaths(active_mask_);
}

LineSensor::IntersectionEvent LineSensor::intersectionEvent()
{
    IntersectionEvent event;
    const bool        side_branch_active = maskActive(active_mask_, LINE_INTERSECTION_LEFT_MASK) ||
                                           maskActive(active_mask_, LINE_INTERSECTION_RIGHT_MASK);
    const uint32_t    now_ms             = to_ms_since_boot(get_absolute_time());

    if (!intersection_pending_)
    {
        if (!side_branch_active)
        {
            intersection_armed_ = true;
            return event;
        }
        if (!intersection_armed_)
            return event;

        intersection_pending_          = true;
        intersection_armed_            = false;
        intersection_window_start_ms_  = now_ms;
        intersection_active_peak_mask_ = active_mask_;
        intersection_raw_peak_mask_    = raw_data_;
    }
    else
    {
        intersection_active_peak_mask_ |= active_mask_;
        intersection_raw_peak_mask_ |= raw_data_;
    }

    const uint32_t elapsed_ms = now_ms - intersection_window_start_ms_;
    if (elapsed_ms < LINE_INTERSECTION_WINDOW_MS)
        return event;

    intersection_pending_ = false;

    event.valid              = true;
    event.paths_mask         = classifyPaths(intersection_active_peak_mask_);
    event.raw_peak_mask      = intersection_raw_peak_mask_;
    event.elapsed_ms         = elapsed_ms;
    last_intersection_event_ = event;

    if (!side_branch_active)
        intersection_armed_ = true;

    return event;
}

LineSensor::IntersectionEvent LineSensor::lastIntersectionEvent() const
{
    return last_intersection_event_;
}

bool LineSensor::detect_intersection()
{
    return intersectionEvent().valid;
}

uint8_t LineSensor::rawByte() const
{
    return raw_data_;
}

uint8_t LineSensor::activeMask() const
{
    return active_mask_;
}
