#ifndef DRIVERS_LINE_SENSOR_H
#define DRIVERS_LINE_SENSOR_H

#include <cstdint>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "config/config.h"

/**
 * @brief I2C driver for the YahBoom 8-channel infrared line patrol sensor
 *
 * Reads an 8-bit bitmask over I2C where each bit represents one IR channel.
 * Bit = 1 means that channel detects the line (contrasting surface).
 * Bit 0 = leftmost sensor, Bit 7 = rightmost sensor.
 *
 * Provides weighted-average position and intersection detection.
 */
class LineSensor
{
  public:
    /**
     * @brief Constructs line sensor with I2C configuration
     * @param i2c I2C instance (e.g. i2c0)
     * @param sda_pin SDA GPIO pin
     * @param scl_pin SCL GPIO pin
     * @param addr 7-bit I2C address of the sensor
     */
    LineSensor(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint8_t addr = LINE_SENSOR_I2C_ADDR);

    /**
     * @brief Initializes I2C peripheral and GPIO pins
     */
    void init();

    /**
     * @brief Reads the 8-channel sensor data over I2C
     * Updates internal bitmask. Call this each control loop tick.
     */
    void read();

    /**
     * @brief Returns the raw 8-bit sensor bitmask from last read
     * Bit 0 = leftmost, Bit 7 = rightmost. 1 = on line.
     */
    uint8_t getSensorData() const;

    /**
     * @brief Computes weighted-average position error
     * @return Position in range [-3.5, +3.5]. 0.0 = centered on line.
     *         Negative = line is to the left, positive = line is to the right.
     */
    float getPosition() const;

    /**
     * @brief Returns true if any sensor detects the line
     */
    bool onLine() const;

    /**
     * @brief Detects a perpendicular intersection
     *
     * Returns true if the 2 leftmost sensors OR the 2 rightmost sensors
     * are both active, indicating a perpendicular line crossing.
     */
    bool detectIntersection() const;

  private:
    i2c_inst_t* i2c_;
    uint        sda_pin_;
    uint        scl_pin_;
    uint8_t     addr_;
    uint8_t     sensor_data_ = 0;
};

#endif // DRIVERS_LINE_SENSOR_H
