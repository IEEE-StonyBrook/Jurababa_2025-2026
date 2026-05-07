/**
 * @file boot.h
 * @brief Boot-time mode, sensor, and status LED selection.
 */
#ifndef CONFIG_BOOT_H
#define CONFIG_BOOT_H

enum class BootModeSelection
{
    Prompt,
    Competition,
    CLI,
    DriverLab
};

enum class BootSensorSelection
{
    Prompt,
    ToF,
    LineSensor
};

// Set either value to Prompt to restore the boot-time keyboard picker.
// Competition boots directly into the gesture-driven COMP loop.
#define BOOT_MODE_SELECTION   BootModeSelection::CLI
#define BOOT_SENSOR_SELECTION BootSensorSelection::ToF

#define BOOT_MODE_PROMPT_TIMEOUT_MS   3000u
#define BOOT_SENSOR_PROMPT_TIMEOUT_MS 3000u

// Waveshare RP2040-Zero onboard LED: WS2812/NeoPixel data input on GPIO16.
#define WAVESHARE_ZERO_LED_ENABLE 1
#define PIN_WAVESHARE_WS2812      16
#define BOOT_LED_FLASH_COUNT      3
#define BOOT_LED_FLASH_ON_MS      120u
#define BOOT_LED_FLASH_OFF_MS     120u
#define BOOT_LED_BRIGHTNESS       24u

#endif // CONFIG_BOOT_H
