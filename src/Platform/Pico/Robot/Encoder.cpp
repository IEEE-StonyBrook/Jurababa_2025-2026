/******************************************************************************
 * @file    Encoder.cpp
 * @brief   Implementation of AS5048A encoder driver for Raspberry Pi Pico.
 ******************************************************************************/

#include "../../../Include/Platform/Pico/Robot/Encoder.h"
#include "../../../Include/Common/LogSystem.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

// AS5048A constants
#define AS5048A_CPR        16384    // 14-bit resolution
#define AS5048A_REG_ANGLE  0x3FFF   // angle register
#define AS5048A_RW_BIT     0x4000   // R/W = 1 for read

Encoder::Encoder(spi_inst_t* spi_port, uint cs_gpio)
    : spi_(spi_port), cs_gpio_(cs_gpio), last_raw_(0), tick_count_(0) {}

void Encoder::init() {
    // CS pin setup
    gpio_init(cs_gpio_);
    gpio_set_dir(cs_gpio_, GPIO_OUT);
    gpio_put(cs_gpio_, 1); // deselect

    last_raw_ = readRawAngle();
    tick_count_ = 0;
}

void Encoder::select() {
    gpio_put(cs_gpio_, 0);
}

void Encoder::deselect() {
    gpio_put(cs_gpio_, 1);
}

// ---- Parity calculation (even parity for bits 0..13 + R/W bit) ----
uint8_t Encoder::calcParity(uint16_t value) {
    uint8_t cnt = 0;
    for (int i = 0; i < 15; i++) {
        if (value & (1 << i)) cnt++;
    }
    return cnt & 0x1; // return 1 if odd, 0 if even
}

// ---- SPI transfer ----
uint16_t Encoder::spiTransfer16(uint16_t data) {
    uint8_t tx[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    uint8_t rx[2] = { 0, 0 };
    select();
    spi_write_read_blocking(spi_, tx, rx, 2);
    deselect();
    return ((uint16_t)rx[0] << 8) | rx[1];
}

// ---- Read raw 14-bit angle ----
uint16_t Encoder::readRawAngle() {
    uint16_t command = AS5048A_REG_ANGLE | AS5048A_RW_BIT;
    command |= (calcParity(command) << 15);

    // issue command
    spiTransfer16(command);
    // response
    uint16_t result = spiTransfer16(0xFFFF);

    // check error flag
    if (result & 0x4000) {
        LOG_ERROR("[Encoder] Error flag set result=0x" + std::to_string(result));
        return 0;
    }

    // check parity
    uint8_t parity = calcParity(result & 0x7FFF);
    if (((result >> 15) & 0x1) != parity) {
        LOG_ERROR("[Encoder] Parity error result=0x" + std::to_string(result));
        return 0;
    }

    return result & 0x3FFF;
}

// ---- Convert to degrees ----
float Encoder::readDegrees() {
    uint16_t raw = readRawAngle();
    return (static_cast<float>(raw) * 360.0f) / AS5048A_CPR;
}

// ---- Tick counter with wrap-around ----
int32_t Encoder::getTickCount() {
    uint16_t raw = readRawAngle();
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(last_raw_);

    // Handle wrap-around on 14-bit counter
    if (delta > (AS5048A_CPR / 2))  delta -= AS5048A_CPR;
    if (delta < -(AS5048A_CPR / 2)) delta += AS5048A_CPR;

    tick_count_ += delta;
    last_raw_ = raw;
    return tick_count_;
}

// ---- Reset counter ----
void Encoder::reset() {
    last_raw_ = readRawAngle();
    tick_count_ = 0;
}