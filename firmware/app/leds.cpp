#include "app/leds.h"

#include <cstdint>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "config/boot.h"
#include "ws2812.pio.h"

namespace
{
constexpr uint8_t kStageBrightness = 32;
constexpr uint8_t kAbortBrightness = 48;
constexpr uint8_t kGoalBrightness  = 64;

// WS2812 wants GRB ordering on the wire. Construct the 24-bit word with the
// PIO program's expected packing (pio_sm_put_blocking shifts left by 8).
uint32_t grbPixel(uint8_t r, uint8_t g, uint8_t b)
{
    return (static_cast<uint32_t>(g) << 16u) | (static_cast<uint32_t>(r) << 8u) |
           static_cast<uint32_t>(b);
}

void putPixel(uint32_t grb)
{
#if WAVESHARE_ZERO_LED_ENABLE
    static bool initialised = false;
    static PIO  pio         = pio1;
    static uint sm          = 0;

    if (!initialised)
    {
        const uint offset = pio_add_program(pio, &ws2812_program);
        ws2812_program_init(pio, sm, offset, PIN_WAVESHARE_WS2812, 800000.0f, false);
        initialised = true;
    }

    pio_sm_put_blocking(pio, sm, grb << 8u);
#else
    (void)grb;
#endif
}
} // namespace

namespace stage_led
{

void flashBoot()
{
#if WAVESHARE_ZERO_LED_ENABLE
    const uint8_t level = static_cast<uint8_t>(BOOT_LED_BRIGHTNESS);
    for (int i = 0; i < BOOT_LED_FLASH_COUNT; ++i)
    {
        putPixel(grbPixel(0, level, 0));
        sleep_ms(BOOT_LED_FLASH_ON_MS);
        putPixel(0);
        if (i < BOOT_LED_FLASH_COUNT - 1)
            sleep_ms(BOOT_LED_FLASH_OFF_MS);
    }
#endif
}

void setArmed()
{
    putPixel(grbPixel(kStageBrightness, kStageBrightness, kStageBrightness));
}

void setStage(int stage)
{
    switch (stage)
    {
        case 1: // search-to-goal: yellow
            putPixel(grbPixel(kStageBrightness, kStageBrightness, 0));
            return;
        case 2: // return-to-start: cyan
            putPixel(grbPixel(0, kStageBrightness, kStageBrightness));
            return;
        case 3: // fast cardinal: green
            putPixel(grbPixel(0, kStageBrightness, 0));
            return;
        case 5: // fast diagonal: magenta
            putPixel(grbPixel(kStageBrightness, 0, kStageBrightness));
            return;
        default:
            setIdle();
            return;
    }
}

void setGoalReached()
{
    putPixel(grbPixel(0, kGoalBrightness, kGoalBrightness));
}

void flashAborted()
{
    for (int i = 0; i < 3; ++i)
    {
        putPixel(grbPixel(kAbortBrightness, 0, 0));
        sleep_ms(120);
        putPixel(0);
        if (i < 2)
            sleep_ms(80);
    }
}

void setPaused()
{
    putPixel(grbPixel(kAbortBrightness, 0, 0));
}

void setCheeseHunting()
{
    putPixel(grbPixel(kAbortBrightness, 0, 0));
}

void setBeaconFound()
{
    putPixel(grbPixel(0, kGoalBrightness, 0));
}

void setIdle()
{
    putPixel(0);
}

} // namespace stage_led
