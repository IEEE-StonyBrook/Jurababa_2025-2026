#include "drivers/battery.h"

#include "common/breadcrumb.h"
#include "common/log.h"
#include "config/config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

Battery::Battery(uint8_t adc_pin, float r1_ohms, float r2_ohms)
    : adc_pin_(adc_pin), adc_channel_(adc_pin - 26), r1_ohms_(r1_ohms), r2_ohms_(r2_ohms),
      divider_ratio_((r1_ohms + r2_ohms) / r2_ohms), reading_index_(0), reading_sum_(0),
      buffer_filled_(false), sagging_(false)
{
    for (int i = 0; i < AVERAGE_SAMPLES; i++)
    {
        raw_readings_[i] = 0;
    }
}

void Battery::begin()
{
    adc_init();
    adc_gpio_init(adc_pin_);
}

void Battery::update()
{
    adc_select_input(adc_channel_);
    uint16_t new_reading = adc_read();

    reading_sum_ -= raw_readings_[reading_index_];
    reading_sum_ += new_reading;
    raw_readings_[reading_index_] = new_reading;

    reading_index_++;
    if (reading_index_ >= AVERAGE_SAMPLES)
    {
        reading_index_ = 0;
        buffer_filled_ = true;
    }

    // The instantaneous reading, not voltage(): a 20 ms moving average smooths away
    // the brief collapse that a stalled motor causes, which is the event we are after.
    const float instant_v = voltsFromAdc(static_cast<float>(new_reading));
    Breadcrumb::noteVoltage(instant_v);

    // Edge-triggered so a sag logs once, not 500 times a second. Under 1 V means no
    // pack is connected (USB-only), which is not a fault worth reporting.
    const bool present = instant_v > 1.0f;
    if (present && !sagging_ && instant_v < BATTERY_SAG_WARN_V)
    {
        sagging_ = true;
        LOG_WARNING("Battery sagging under load: " << instant_v << " V (warn below "
                                                   << BATTERY_SAG_WARN_V << " V)");
    }
    else if (sagging_ && (!present || instant_v > BATTERY_SAG_WARN_V + BATTERY_SAG_CLEAR_V))
    {
        sagging_ = false;
        LOG_INFO("Battery recovered: " << instant_v << " V");
    }
}

float Battery::voltsFromAdc(float adc_counts) const
{
    return (adc_counts / ADC_MAX) * ADC_REF_VOLTAGE * divider_ratio_;
}

float Battery::voltage() const
{
    int sample_count = buffer_filled_ ? AVERAGE_SAMPLES : (reading_index_ > 0 ? reading_index_ : 1);
    float avg_adc    = static_cast<float>(reading_sum_) / static_cast<float>(sample_count);
    return voltsFromAdc(avg_adc);
}

uint16_t Battery::raw_adc() const
{
    int idx = (reading_index_ == 0) ? (AVERAGE_SAMPLES - 1) : (reading_index_ - 1);
    return raw_readings_[idx];
}
