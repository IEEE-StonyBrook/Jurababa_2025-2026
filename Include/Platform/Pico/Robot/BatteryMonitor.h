#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

#include <cstdint>

/**
 * @class BatteryMonitor
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
class BatteryMonitor {
  public:
    /**
     * @brief Constructs a BatteryMonitor for the specified ADC pin.
     *
     * @param adc_pin GPIO pin number (26-29 for ADC0-ADC3).
     * @param r1_ohms Upper resistor in voltage divider (default 10000 ohms).
     * @param r2_ohms Lower resistor in voltage divider (default 5100 ohms).
     */
    BatteryMonitor(uint8_t adc_pin = 26, float r1_ohms = 10000.0f, float r2_ohms = 5100.0f);

    /**
     * @brief Initializes the ADC hardware. Call once during setup.
     */
    void init();

    /**
     * @brief Takes a new ADC reading and updates the moving average.
     *
     * Call this periodically (e.g., every 100ms) from your main loop.
     * Non-blocking - returns immediately after reading.
     */
    void update();

    /**
     * @brief Returns the smoothed battery voltage.
     *
     * @return float Battery voltage in volts (smoothed with 10-sample moving average).
     */
    float getVoltage() const;

    /**
     * @brief Returns the raw ADC value (0-4095) of the most recent reading.
     *
     * @return uint16_t Raw 12-bit ADC value.
     */
    uint16_t getRawADC() const;

    /**
     * @brief Checks if battery voltage is below a critical threshold.
     *
     * @param threshold_volts Voltage threshold (default 6.0V for 2S LiPo).
     * @return true if battery is low, false otherwise.
     */
    bool isLowBattery(float threshold_volts = 6.0f) const;

  private:
    static constexpr int    AVERAGE_SAMPLES = 10;      ///< Number of samples for moving average.
    static constexpr float  ADC_MAX         = 4095.0f; ///< 12-bit ADC maximum value.
    static constexpr float  ADC_REF_VOLTAGE = 3.3f;    ///< ADC reference voltage.

    uint8_t  adc_pin_;                        ///< GPIO pin for ADC input.
    uint8_t  adc_channel_;                    ///< ADC channel (0-3).
    float    r1_ohms_;                        ///< Upper resistor value.
    float    r2_ohms_;                        ///< Lower resistor value.
    float    divider_ratio_;                  ///< Precomputed (R1+R2)/R2 for efficiency.

    uint16_t raw_readings_[AVERAGE_SAMPLES];  ///< Circular buffer for moving average.
    uint8_t  reading_index_;                  ///< Current index in circular buffer.
    uint32_t reading_sum_;                    ///< Running sum for efficient average calculation.
    bool     buffer_filled_;                  ///< True once all buffer slots have been written.
};

#endif  // BATTERYMONITOR_H
