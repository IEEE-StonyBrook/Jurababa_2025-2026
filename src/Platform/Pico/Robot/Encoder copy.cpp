/******************************************************************************
 * @file    Encoder.cpp
 * @brief   Implementation of AS5048A encoder driver.
 ******************************************************************************/

#include "../../../Include/Platform/Pico/Robot/Encoder copy.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define AS5048A_CMD_READ   0x4000
#define AS5048A_REG_ANGLE  0x3FFF

Encoder::Encoder(spi_inst_t* spi_port, uint cs_gpio)
    : spi_(spi_port), cs_gpio_(cs_gpio), last_raw_(0), tick_count_(0) {}

void Encoder::init() {
  gpio_init(cs_gpio_);
  gpio_set_dir(cs_gpio_, GPIO_OUT);
  gpio_put(cs_gpio_, 1);
}

uint16_t Encoder::transfer(uint16_t command) {
  uint8_t txbuf[2] = {
      static_cast<uint8_t>((command >> 8) & 0xFF),
      static_cast<uint8_t>(command & 0xFF)};
  uint8_t rxbuf[2] = {0, 0};

  select();
  spi_write_read_blocking(spi_, txbuf, rxbuf, 2);
  deselect();

  return (static_cast<uint16_t>(rxbuf[0]) << 8) | rxbuf[1];
}

uint16_t Encoder::readRaw() {
  uint16_t raw = transfer(AS5048A_CMD_READ | AS5048A_REG_ANGLE);
  return raw & 0x3FFF; // 14-bit value
}

float Encoder::readDegrees() {
  uint16_t raw = readRaw();
  return (static_cast<float>(raw) * 360.0f) / 16384.0f;
}

int32_t Encoder::getTickCount() {
  uint16_t raw = readRaw();

  // Compute signed delta with wrap-around correction
  int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(last_raw_);
  if (delta > 8192) delta -= 16384;
  if (delta < -8192) delta += 16384;

  tick_count_ += delta;
  last_raw_ = raw;

  return tick_count_;
}