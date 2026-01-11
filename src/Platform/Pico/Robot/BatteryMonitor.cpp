#include "Platform/Pico/Robot/BatteryMonitor.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

BatteryMonitor::BatteryMonitor(uint8_t adc_pin, float r1_ohms, float r2_ohms)
    : adc_pin_(adc_pin),
      adc_channel_(adc_pin - 26),  // ADC0 = GPIO26, ADC1 = GPIO27, etc.
      r1_ohms_(r1_ohms),
      r2_ohms_(r2_ohms),
      divider_ratio_((r1_ohms + r2_ohms) / r2_ohms),
      reading_index_(0),
      reading_sum_(0),
      buffer_filled_(false)
{
    // Initialize buffer to zero
    for (int i = 0; i < AVERAGE_SAMPLES; i++)
    {
        raw_readings_[i] = 0;
    }
}

void BatteryMonitor::init()
{
    // Initialize ADC hardware (only needs to be done once globally)
    adc_init();

    // Configure the GPIO pin for ADC input (disable digital functions)
    adc_gpio_init(adc_pin_);
}

void BatteryMonitor::update()
{
    // Select the ADC channel
    adc_select_input(adc_channel_);

    // Read the ADC (non-blocking, ~2us)
    uint16_t new_reading = adc_read();

    // Update running sum: subtract old value, add new value
    reading_sum_ -= raw_readings_[reading_index_];
    reading_sum_ += new_reading;

    // Store new reading in circular buffer
    raw_readings_[reading_index_] = new_reading;

    // Advance index with wraparound
    reading_index_++;
    if (reading_index_ >= AVERAGE_SAMPLES)
    {
        reading_index_ = 0;
        buffer_filled_ = true;  // All buffer slots now contain valid readings
    }
}

float BatteryMonitor::getVoltage() const
{
    // Calculate average ADC value
    int     sample_count = buffer_filled_ ? AVERAGE_SAMPLES : (reading_index_ > 0 ? reading_index_ : 1);
    float   avg_adc      = static_cast<float>(reading_sum_) / static_cast<float>(sample_count);

    // Convert ADC value to voltage at divider output
    float v_adc = (avg_adc / ADC_MAX) * ADC_REF_VOLTAGE;

    // Scale up by divider ratio to get actual battery voltage
    return v_adc * divider_ratio_;
}

uint16_t BatteryMonitor::getRawADC() const
{
    // Return most recent reading (one before current index)
    int idx = (reading_index_ == 0) ? (AVERAGE_SAMPLES - 1) : (reading_index_ - 1);
    return raw_readings_[idx];
}

bool BatteryMonitor::isLowBattery(float threshold_volts) const
{
    return getVoltage() < threshold_volts;
}
