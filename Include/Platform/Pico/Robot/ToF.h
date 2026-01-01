#ifndef TOF_H
#define TOF_H

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
     * @brief Returns measured distance to nearest object
     * @return Distance in millimeters (or multicore snapshot if enabled)
     */
    float getToFDistanceFromWallMM();

  private:
    /**
     * @brief Configures XSHUT pin as output
     * @param xshut_pin GPIO pin number
     */
    void setupXSHUTPin(int xshut_pin);

    /**
     * @brief Resets sensor by pulling XSHUT low
     * @param xshut_pin GPIO pin number
     */
    void resetSensor(int xshut_pin);

    /**
     * @brief Initializes sensor and assigns unique I2C address
     * @param xshut_pin GPIO pin number
     * @param sensor_position Position character for address selection
     */
    void initializeSensor(int xshut_pin, char sensor_position);

    /**
     * @brief Configures continuous ranging mode with timing budget
     */
    void setupContinuousRanging();

    // VL53L0X sensor device structure
    VL53L0X_Dev_t sensor_device_;

    // Sensor position identifier ('l', 'f', 'r')
    char sensor_position_;
};

#endif