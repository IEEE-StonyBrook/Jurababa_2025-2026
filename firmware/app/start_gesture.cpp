#include "app/start_gesture.h"

#include <cstdio>

#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/multicore.h"

namespace
{
enum class WaveState
{
    ARMED_HIGH, // No hand present yet; waiting for low.
    WAVE_LOW    // Hand detected close; waiting for it to move clear.
};

bool peekSerialG()
{
    int c = getchar_timeout_us(0);
    return c == 'G' || c == 'g';
}
} // namespace

StartTrigger waitForStartGesture(Bluetooth* bt, bool front_tof_available, uint32_t low_mm,
                                 uint32_t high_mm)
{
    WaveState state = WaveState::ARMED_HIGH;

    while (true)
    {
        if (peekSerialG())
            return StartTrigger::SERIAL_G;

        if (bt != nullptr && bt->hasCommand())
        {
            Bluetooth::Command cmd = bt->command();
            if (cmd == Bluetooth::Command::START)
                return StartTrigger::BT_START;
            if (cmd == Bluetooth::Command::HALT)
                return StartTrigger::CANCELLED;
            // Other commands (BATTERY, RESET) are ignored here — they belong to
            // the Cli command loop, not the start-gesture handshake.
        }

        if (front_tof_available)
        {
            SensorData snap;
            SensorHub::snapshot(snap);
            int16_t mm = snap.tof_front_mm;

            // Treat 0 (VL53L0X "invalid") as "far" so a sensor dropout never
            // looks like a hand-close transition.
            uint32_t distance = (mm > 0) ? static_cast<uint32_t>(mm) : high_mm + 1;

            switch (state)
            {
                case WaveState::ARMED_HIGH:
                    if (distance < low_mm)
                        state = WaveState::WAVE_LOW;
                    break;
                case WaveState::WAVE_LOW:
                    if (distance > high_mm)
                        return StartTrigger::TOF_WAVE;
                    break;
            }
        }

        sleep_ms(20);
    }
}
