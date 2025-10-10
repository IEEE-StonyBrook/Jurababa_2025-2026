#ifndef BATTERY_H
#define BATTERY_H

#include "hardware/adc.h"

class Battery
{
  public:
    Battery(int senseGpio, float refVoltage, float resistorTop, float resistorBottom);
    float readVoltage();

  private:
    int   adcChannel;    // ADC channel (0–3).
    float referenceVolt; // ADC reference voltage in volts.
    float dividerTop;    // Top resistor of divider in ohms.
    float dividerBottom; // Bottom resistor of divider in ohms.
};

#endif