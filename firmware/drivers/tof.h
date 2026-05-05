#ifndef DRIVERS_TOF_H
#define DRIVERS_TOF_H

#include "vl53l0x_api_rp2040/core/inc/vl53l0x_api.h"
#include "vl53l0x_api_rp2040/platform/inc/vl53l0x_rp2040.h"

/**
 * @brief Time-of-Flight (ToF) distance sensor interface for VL53L0X
 *
 * Provides interface to VL53L0X laser ranging sensor for wall detection.
 * Supports multiple sensors on same I2C bus via XSHUT pin control and
 * address remapping. Operates in continuous ranging mode with configurable
 * timing budget.
 */
class ToF
{
  public:
    /**
     * @brief Constructs ToF sensor interface
     * @param xshut_pin GPIO pin connected to sensor XSHUT (shutdown) pin
     * @param sensor_position Position identifier ('l'=left, 'f'=front, 'r'=right)
     */
    ToF(int xshut_pin, char sensor_position);

    /**
     * @brief Returns measured distance to nearest object.
     *
     * Reads hardware directly. On a valid measurement, returns the measured
     * range in millimeters. On an invalid measurement (RangeStatus != 0,
     * e.g., out-of-range or signal too low), returns the VL53L0X
     * out-of-range sentinel from config.
     *
     * @return Distance in millimeters.
     */
    float get_distance();

  private:
    void setup_xshut_pin(int xshut_pin);
    void reset_sensor(int xshut_pin);
    void initialize_sensor(int xshut_pin, char sensor_position);
    void setup_continuous_ranging();

    VL53L0X_Dev_t sensor_device_;
    char          sensor_position_;
};

#endif
