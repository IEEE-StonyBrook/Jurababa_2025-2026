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
 *
 * No per-device offset calibration is run. The chip's mm output carries
 * a 10-30 mm per-unit bias (cover, mounting, factory variance); that
 * bias is absorbed at compile time via the per-side scale factors in
 * config/sensors.h (TOF_LEFT_SCALE / TOF_RIGHT_SCALE), mirroring the
 * UKMARS mazerunner-core SIDE_NOMINAL pattern. Wall-detection thresholds
 * are binary classifiers; absorbing the same bias in their #defines is
 * also acceptable.
 */
class ToF
{
  public:
    ToF(int xshut_pin, char sensor_position);

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
