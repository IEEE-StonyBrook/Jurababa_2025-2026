#include "app/start_gesture.h"

#include <cstdio>

#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/multicore.h"
#include "config/sensors.h"

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

const char* stateName(WaveState state)
{
    return state == WaveState::ARMED_HIGH ? "ARMED_HIGH" : "WAVE_LOW";
}
} // namespace

StartTrigger waitForStartGesture(Bluetooth* bt, bool front_tof_available, uint32_t low_mm,
                                 uint32_t high_mm)
{
    WaveState state          = WaveState::ARMED_HIGH;
    uint32_t  last_report_ms = 0;

    while (true)
    {
        if (bt != nullptr)
            bt->drain();

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

            // Treat invalid/open-space readings as "far" so a sensor dropout
            // never looks like a hand-close transition.
            uint32_t distance = (mm > 0 && mm < static_cast<int16_t>(TOF_OUT_OF_RANGE_MM))
                                    ? static_cast<uint32_t>(mm)
                                    : high_mm + 1;

            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_report_ms >= 250)
            {
                printf("Start ToF front=%d mm interpreted=%lu state=%s low=%lu high=%lu\n", mm,
                       static_cast<unsigned long>(distance), stateName(state),
                       static_cast<unsigned long>(low_mm), static_cast<unsigned long>(high_mm));
                last_report_ms = now_ms;
            }

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
