#pragma once
#include <cstdint>

#include "hardware/spi.h"
#include "pico/stdlib.h"

class Encoder {
 public:
  Encoder(uint8_t ADCPin) : ADCPin(ADCPin) {}

  // Angle (0..16383)
  float readRotation();

 private:
  uint8_t ADCPin;
  uint16_t last_raw_ = 0;
  int32_t tick_count_ = 0;
  bool primed_ = false;
};
