#include "../../../Include/Platform/Pico/Robot/Battery.h"

Battery::Battery(int senseGpio, float refVoltage, float resistorTop, float resistorBottom)
    : referenceVolt(refVoltage), dividerTop(resistorTop), dividerBottom(resistorBottom)
{
    adc_init();
    adc_gpio_init(senseGpio);
    adcChannel = senseGpio - 26; // GPIO26->ADC0, GPIO27->ADC1, GPIO28->ADC2.
    adc_select_input(adcChannel);
}

float Battery::readVoltage()
{
    uint16_t rawAdc = adc_read(); // Raw ADC value (0–4095).
    float    senseVoltage =
        (rawAdc * referenceVolt) / (1 << 12); // Voltage at ADC pin (12-bit resolution).
    return senseVoltage * ((dividerTop + dividerBottom) / dividerBottom); // Battery voltage.
}
