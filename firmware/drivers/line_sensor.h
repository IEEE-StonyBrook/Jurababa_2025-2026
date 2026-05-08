#ifndef DRIVERS_LINE_SENSOR_H
#define DRIVERS_LINE_SENSOR_H

#include <cstdint>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "config/config.h"

/**
 * @brief I2C driver for the YahBoom 8-channel infrared line patrol sensor
 *
 * Reads an 8-bit bitmask over I2C where each bit represents one IR channel,
 * then converts it to an active-line mask using LINE_SENSOR_ACTIVE_LOW.
 *
 * Confirmed on Jurababa with LINCON:
 *   bit 0 / X8 = leftmost sensor  = position -3.5
 *   bit 7 / X1 = rightmost sensor = position +3.5
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
    void begin();

    /**
     * @brief Reads the 8-channel sensor data over I2C
     * Updates internal bitmask. Call this each control loop tick.
     * @return true when a new byte was read successfully
     */
    bool read();

    /**
     * @brief Computes weighted-average position error
     * @return Position in range [-3.5, +3.5]. 0.0 = centered on line.
     *         Negative = line is to the left, positive = line is to the right.
     *
     * If the line is not currently visible, returns the last valid position
     * instead of pretending the robot is centered.
     */
    float get_position() const;

    /**
     * @brief Returns true if any sensor detects the line
     */
    bool on_line() const;

    /**
     * @brief Detects a branch or crossing
     *
     * Returns true if the 2 leftmost sensors OR the 2 rightmost sensors
     * are both active, indicating a branch or crossing.
     */
    bool detect_intersection();

    /**
     * @brief Last raw byte read from the Yahboom register.
     */
    uint8_t rawByte() const;

    /**
     * @brief Last raw byte converted through LINE_SENSOR_ACTIVE_LOW.
     */
    uint8_t activeMask() const;

  private:
    void updateDerivedState();

    i2c_inst_t* i2c_;
    uint        sda_pin_;
    uint        scl_pin_;
    uint8_t     addr_;
    uint8_t     raw_data_       = 0;
    uint8_t     active_mask_    = 0;
    float       position_       = 0.0f;
    float       last_position_  = 0.0f;
    bool        position_valid_ = false;
    bool        read_valid_     = false;
};

#endif // DRIVERS_LINE_SENSOR_H
