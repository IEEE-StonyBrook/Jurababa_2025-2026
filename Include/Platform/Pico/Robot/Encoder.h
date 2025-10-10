#ifndef ENCODER_H
#define ENCODER_H

#include "hardware/spi.h"
#include <cstdint>

class Encoder {
public:
    // Constructor: provide SPI instance (e.g., spi1) and CS pin
    Encoder(spi_inst_t* spi_port, uint cs_gpio);

    // Initialize chip select pin and reset counters
    void init();

    // Get the latest angle in degrees [0,360)
    float readDegrees();

    // Get running tick count (increments/decrements across rotations)
    int32_t getTickCount();

    // Reset tick counter and latch current position as reference
    void reset();

    // Read raw 14-bit angle directly (0–16383)
    uint16_t readRawAngle();

private:
    spi_inst_t* spi_;
    uint cs_gpio_;
    uint16_t last_raw_;
    int32_t tick_count_;

    void select();
    void deselect();
    uint16_t spiTransfer16(uint16_t data);
    uint8_t calcParity(uint16_t value);
};

#endif // ENCODER_H