/******************************************************************************
 * @file    Encoder.cpp
 * @brief   Implementation of AS5048A encoder driver.
 ******************************************************************************/

#include "../../../Include/Platform/Pico/Robot/Encoder.h"
#include "../../../Include/Common/LogSystem.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define AS5048A_CMD_READ  0x4000
#define AS5048A_REG_ANGLE 0x3FFF

Encoder::Encoder(spi_inst_t* spi_port, uint cs_gpio)
    : spi_(spi_port), cs_gpio_(cs_gpio), last_raw_(0), tick_count_(0)
{
}

void Encoder::init()
{
    gpio_init(cs_gpio_);
    LOG_DEBUG("Initialized GPIO " + std::to_string(cs_gpio_) + " for Encoder CS");
    gpio_set_dir(cs_gpio_, GPIO_OUT);
    LOG_DEBUG("Set GPIO " + std::to_string(cs_gpio_) + " as OUTPUT for Encoder CS");
    gpio_put(cs_gpio_, 1);
    LOG_DEBUG("Set GPIO " + std::to_string(cs_gpio_) + " HIGH (deselected) for Encoder CS");
}

uint16_t Encoder::transfer(uint16_t command)
{
    uint8_t txbuf[2] = {static_cast<uint8_t>((command >> 8) & 0xFF),
                        static_cast<uint8_t>(command & 0xFF)};
    uint8_t rxbuf[2] = {0, 0};

    LOG_DEBUG("Transferring command: 0x" + std::to_string(command));
    select();
    LOG_DEBUG("Selected encoder (CS LOW)");
    spi_write_read_blocking(spi_, txbuf, rxbuf, 2);
    LOG_DEBUG("Received response: 0x" + std::to_string((static_cast<uint16_t>(rxbuf[0]) << 8) | rxbuf[1]));
    deselect();
    LOG_DEBUG("Deselected encoder (CS HIGH)");

    return (static_cast<uint16_t>(rxbuf[0]) << 8) | rxbuf[1];
}

uint16_t Encoder::readRaw()
{
    LOG_DEBUG("Reading raw encoder value...");
    uint16_t raw = transfer(AS5048A_CMD_READ | AS5048A_REG_ANGLE);
    LOG_DEBUG("Raw encoder value: " + std::to_string(raw));
    return raw & 0x3FFF; // 14-bit value
}

float Encoder::readDegrees()
{
    uint16_t raw = readRaw();
    return (static_cast<float>(raw) * 360.0f) / 16384.0f;
}

int32_t Encoder::getTickCount()
{
    uint16_t raw = readRaw();

    // Compute signed delta with wrap-around correction
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(last_raw_);
    if (delta > 8192)
        delta -= 16384;
    if (delta < -8192)
        delta += 16384;

    tick_count_ += delta;
    last_raw_ = raw;

    return tick_count_;
}

void Encoder::reset()
{
    LOG_DEBUG("Resetting encoder...");
    last_raw_   = readRaw();
    tick_count_ = 0;
    LOG_DEBUG("Encoder reset complete.");
}