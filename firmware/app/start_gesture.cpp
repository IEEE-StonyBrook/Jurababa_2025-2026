#include "app/start_gesture.h"

#include <cstdio>

#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/bootsel_button.h"
#include "common/log.h"
#include "config/sensors.h"
#include "drivers/tof.h"

namespace
{
enum class WaveState
{
    ARMED_HIGH, // No hand present yet; waiting for low.
    WAVE_LOW    // Hand detected close; waiting for it to move clear.
};

const char* stateName(WaveState state)
{
    return state == WaveState::ARMED_HIGH ? "ARMED_HIGH" : "WAVE_LOW";
}

// Hysteresis state machine for one ToF. Caller passes the latest distance
// reading each tick. Returns true on the rising edge (hand left after a
// close-and-clear), at which point the machine self-resets to ARMED_HIGH.
struct WaveMachine
{
    WaveState state = WaveState::ARMED_HIGH;

    bool tick(uint32_t distance_mm, uint32_t low_mm, uint32_t high_mm)
    {
        switch (state)
        {
            case WaveState::ARMED_HIGH:
                if (distance_mm < low_mm)
                    state = WaveState::WAVE_LOW;
                return false;
            case WaveState::WAVE_LOW:
                if (distance_mm > high_mm)
                {
                    state = WaveState::ARMED_HIGH;
                    return true;
                }
                return false;
        }
        return false;
    }
};

bool peekSerialChar(int& out)
{
    int c = getchar_timeout_us(0);
    if (c < 0)
        return false;
    out = c;
    return true;
}

void writeDiagnostic(Bluetooth* bt, const char* text)
{
    if (text == nullptr)
        return;

    std::fputs(text, stdout);
    if (bt != nullptr)
    {
        bt->write(text);
        Log::drainBluetooth();
        bt->drain();
    }
}

// Read a ToF and clamp invalid/open-space readings to "far" (high+1) so a
// sensor dropout never looks like a hand-close transition. Returns the
// interpreted distance for the state machine; the raw mm is returned via
// out_raw_mm for diagnostics.
uint32_t readToFDistance(ToF* tof, uint32_t high_mm, int16_t& out_raw_mm)
{
    out_raw_mm = 0;
    if (tof == nullptr)
        return high_mm + 1;

    const float raw_f = tof->get_distance();
    const bool  valid = raw_f > 0.0f && raw_f < TOF_OUT_OF_RANGE_MM;
    if (!valid)
        return high_mm + 1;

    out_raw_mm = static_cast<int16_t>(raw_f);
    return static_cast<uint32_t>(out_raw_mm);
}
} // namespace

StartTrigger waitForStartGesture(Bluetooth* bt, ToF* front_tof, uint32_t low_mm, uint32_t high_mm)
{
    WaveMachine front;
    uint32_t    last_report_ms = 0;

    while (true)
    {
        if (bt != nullptr)
        {
            Log::drainBluetooth();
            bt->drain();
        }

        int ch = 0;
        if (peekSerialChar(ch) && (ch == 'G' || ch == 'g'))
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

        if (front_tof != nullptr)
        {
            int16_t        raw_mm  = 0;
            const uint32_t front_d = readToFDistance(front_tof, high_mm, raw_mm);

            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_report_ms >= 250)
            {
                char report[160];
                std::snprintf(report, sizeof(report),
                              "Start ToF front=%d mm interpreted=%lu state=%s low=%lu high=%lu\n",
                              raw_mm, static_cast<unsigned long>(front_d), stateName(front.state),
                              static_cast<unsigned long>(low_mm),
                              static_cast<unsigned long>(high_mm));
                writeDiagnostic(bt, report);
                last_report_ms = now_ms;
            }

            if (front.tick(front_d, low_mm, high_mm))
                return StartTrigger::TOF_WAVE;
        }

        sleep_ms(20);
    }
}

StartTrigger waitForCompetitionGesture(Bluetooth* bt, ToF* front_tof, ToF* right_tof, ToF* left_tof,
                                       uint32_t low_mm, uint32_t high_mm)
{
    WaveMachine front;
    WaveMachine right;
    WaveMachine left;
    uint32_t    last_report_ms = 0;

    while (true)
    {
        if (bt != nullptr)
        {
            Log::drainBluetooth();
            bt->drain();
        }

        int ch = 0;
        if (peekSerialChar(ch))
        {
            if (ch == 'G' || ch == 'g')
                return StartTrigger::FRONT_WAVE;
            if (ch == 'H' || ch == 'h')
                return StartTrigger::RIGHT_WAVE;
            if (ch == 'J' || ch == 'j')
                return StartTrigger::LEFT_WAVE;
        }

        if (bt != nullptr && bt->hasCommand())
        {
            Bluetooth::Command cmd = bt->command();
            if (cmd == Bluetooth::Command::START)
                return StartTrigger::FRONT_WAVE;
            if (cmd == Bluetooth::Command::HALT)
                return StartTrigger::CANCELLED;
        }

        // BOOTSEL while armed exits COMP mode without needing a phone or
        // laptop. Mirrors the in-motion abort path so the operator only has
        // one button to remember.
        if (abortRequested())
            return StartTrigger::CANCELLED;

        int16_t  raw_front = 0;
        int16_t  raw_right = 0;
        int16_t  raw_left  = 0;
        uint32_t front_d   = readToFDistance(front_tof, high_mm, raw_front);
        uint32_t right_d   = readToFDistance(right_tof, high_mm, raw_right);
        uint32_t left_d    = readToFDistance(left_tof, high_mm, raw_left);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_report_ms >= 500)
        {
            char report[200];
            std::snprintf(report, sizeof(report),
                          "COMP gesture front=%d(%s) right=%d(%s) left=%d(%s) low=%lu high=%lu\n",
                          raw_front, stateName(front.state), raw_right, stateName(right.state),
                          raw_left, stateName(left.state), static_cast<unsigned long>(low_mm),
                          static_cast<unsigned long>(high_mm));
            writeDiagnostic(bt, report);
            last_report_ms = now_ms;
        }

        // Run all three machines every tick. Same-tick tie-break order is
        // front > right > left, but in practice only one machine fires per
        // gesture because each sensor's range covers a different region of
        // space relative to the robot.
        const bool front_fired = front_tof != nullptr && front.tick(front_d, low_mm, high_mm);
        const bool right_fired = right_tof != nullptr && right.tick(right_d, low_mm, high_mm);
        const bool left_fired  = left_tof != nullptr && left.tick(left_d, low_mm, high_mm);

        if (front_fired)
            return StartTrigger::FRONT_WAVE;
        if (right_fired)
            return StartTrigger::RIGHT_WAVE;
        if (left_fired)
            return StartTrigger::LEFT_WAVE;

        sleep_ms(20);
    }
}
