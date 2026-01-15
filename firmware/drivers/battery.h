#ifndef DRIVERS_BATTERY_H
#define DRIVERS_BATTERY_H

#include <cstdint>

/**
 * @brief Non-blocking battery voltage monitor using ADC with moving average filter.
 *
 * Uses a voltage divider (R1=10k, R2=5.1k) to scale battery voltage
 * to the Pico's 3.3V ADC range. Applies a 10-sample moving average
 * for noise reduction.
 *
 * Voltage divider math:
 *   V_adc = V_battery * R2 / (R1 + R2)
 *   V_battery = V_adc * (R1 + R2) / R2
 */
class Battery
{
  public:
    /**
     * @brief Constructs a Battery monitor for the specified ADC pin.
     * @param adc_pin GPIO pin number (26-29 for ADC0-ADC3).
     * @param r1_ohms Upper resistor in voltage divider (default 10000 ohms).
     * @param r2_ohms Lower resistor in voltage divider (default 5100 ohms).
     */
    Battery(uint8_t adc_pin = 26, float r1_ohms = 10000.0f, float r2_ohms = 5100.0f);

    /**
     * @brief Initializes the ADC hardware. Call once during setup.
     */
    void init();

    /**
     * @brief Takes a new ADC reading and updates the moving average.
     * Call this periodically (e.g., every 100ms) from your main loop.
     */
    void update();

    /**
     * @brief Returns the smoothed battery voltage.
     * @return Battery voltage in volts (smoothed with 10-sample moving average).
     */
    float voltage() const;

    /**
     * @brief Returns the raw ADC value (0-4095) of the most recent reading.
     * @return Raw 12-bit ADC value.
     */
    uint16_t rawADC() const;

    /**
     * @brief Checks if battery voltage is below a critical threshold.
     * @param threshold_volts Voltage threshold (default 6.0V for 2S LiPo).
     * @return true if battery is low, false otherwise.
     */
    bool isLow(float threshold_volts = 6.0f) const;

  private:
    static constexpr int   AVERAGE_SAMPLES = 10;
    static constexpr float ADC_MAX         = 4095.0f;
    static constexpr float ADC_REF_VOLTAGE = 3.3f;

    uint8_t adc_pin_;
    uint8_t adc_channel_;
    float   r1_ohms_;
    float   r2_ohms_;
    float   divider_ratio_;

    uint16_t raw_readings_[AVERAGE_SAMPLES];
    uint8_t  reading_index_;
    uint32_t reading_sum_;
    bool     buffer_filled_;
};

#endif
