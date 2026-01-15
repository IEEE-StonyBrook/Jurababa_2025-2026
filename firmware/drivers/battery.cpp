#include "drivers/battery.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

Battery::Battery(uint8_t adc_pin, float r1_ohms, float r2_ohms)
    : adc_pin_(adc_pin), adc_channel_(adc_pin - 26), r1_ohms_(r1_ohms), r2_ohms_(r2_ohms),
      divider_ratio_((r1_ohms + r2_ohms) / r2_ohms), reading_index_(0), reading_sum_(0),
      buffer_filled_(false)
{
    for (int i = 0; i < AVERAGE_SAMPLES; i++)
    {
        raw_readings_[i] = 0;
    }
}

void Battery::init()
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
}

float Battery::voltage() const
{
    int sample_count = buffer_filled_ ? AVERAGE_SAMPLES : (reading_index_ > 0 ? reading_index_ : 1);
    float avg_adc    = static_cast<float>(reading_sum_) / static_cast<float>(sample_count);
    float v_adc      = (avg_adc / ADC_MAX) * ADC_REF_VOLTAGE;
    return v_adc * divider_ratio_;
}

uint16_t Battery::rawADC() const
{
    int idx = (reading_index_ == 0) ? (AVERAGE_SAMPLES - 1) : (reading_index_ - 1);
    return raw_readings_[idx];
}

bool Battery::isLow(float threshold_volts) const
{
    return voltage() < threshold_volts;
}
