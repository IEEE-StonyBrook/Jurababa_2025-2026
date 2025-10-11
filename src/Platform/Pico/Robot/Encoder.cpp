#include "../../../Include/Platform/Pico/Robot/Encoder.h"

#include <cstdio>

#include "../../../Include/Common/LogSystem.h"

float Encoder::readRotation() {
  // Read raw angle from encoder (0..16383)
  uint16_t raw = 0;
  adc_select_input(ADCPin);
  raw = adc_read();

  // Calculate total angle in degrees
  float angle =
      ((float)raw / 4096.0f) * 360.0f;  // 0..16383 maps to 0..360 degrees
  return angle;
}