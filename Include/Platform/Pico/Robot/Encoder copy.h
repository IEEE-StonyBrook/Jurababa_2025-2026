/******************************************************************************
 * @file    Encoder.h
 * @brief   Driver for AS5048A magnetic rotary encoder via SPI.
 ******************************************************************************/

#ifndef ENCODER_H
#define ENCODER_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <cstdint>


class Encoder
{
  public:
    Encoder(spi_inst_t* spi_port, uint cs_gpio);

    void init();

    /**
     * @brief Read the raw 14-bit angle (0–16383).
     */
    uint16_t readRaw();

    /**
     * @brief Read angle in degrees (0–360).
     */
    float readDegrees();

    /**
     * @brief Get accumulated tick count (like a quadrature encoder).
     *
     * Each step of the AS5048A (1/16384 of a rev) is treated as 1 tick.
     */
    int32_t getTickCount();

  private:
    spi_inst_t* spi_;
    uint        cs_gpio_;

    uint16_t last_raw_;   ///< Last raw angle reading (0–16383)
    int32_t  tick_count_; ///< Accumulated ticks

    uint16_t transfer(uint16_t command);

    inline void select() { gpio_put(cs_gpio_, 0); }
    inline void deselect() { gpio_put(cs_gpio_, 1); }
};

#endif