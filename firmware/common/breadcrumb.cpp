#include "common/breadcrumb.h"

#include <cstdio>
#include <cstring>

#include "common/log.h"

#ifdef PICO_BUILD
#include "config/config.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/watchdog.h"
#include "pico/platform/sections.h"
#include "pico/time.h"
#endif

namespace
{
constexpr uint32_t kMagic = 0x4A555242; // 'JURB'

#ifdef PICO_BUILD
struct Live
{
    uint32_t magic;
    uint32_t uptime_ms;
    float    voltage_min;
    float    voltage_last;
    char     last_log[48];
};

// Must not land in .bss: crt0 zeroes that, which would erase the history of the
// very reset we are trying to explain.
Live __uninitialized_ram(live_crumb);
#endif
} // namespace

Breadcrumb::Snapshot Breadcrumb::previous_ = {};

const Breadcrumb::Snapshot& Breadcrumb::previous()
{
    return previous_;
}

void Breadcrumb::begin()
{
#ifdef PICO_BUILD
    previous_.valid = (live_crumb.magic == kMagic);
    if (previous_.valid)
    {
        previous_.uptime_ms    = live_crumb.uptime_ms;
        previous_.voltage_min  = live_crumb.voltage_min;
        previous_.voltage_last = live_crumb.voltage_last;
        std::memcpy(previous_.last_log, live_crumb.last_log, sizeof(previous_.last_log));
        previous_.last_log[sizeof(previous_.last_log) - 1] = '\0';
    }

    live_crumb.magic        = kMagic;
    live_crumb.uptime_ms    = 0;
    live_crumb.voltage_min  = 0.0f; // 0 means "no reading taken yet"
    live_crumb.voltage_last = 0.0f;
    live_crumb.last_log[0]  = '\0';
#endif
}

void Breadcrumb::noteVoltage(float volts)
{
#ifdef PICO_BUILD
    live_crumb.voltage_last = volts;
    if (live_crumb.voltage_min <= 0.0f || volts < live_crumb.voltage_min)
        live_crumb.voltage_min = volts;
#else
    (void)volts;
#endif
}

void Breadcrumb::noteActivity(const char* text)
{
#ifdef PICO_BUILD
    if (text == nullptr)
        return;

    std::strncpy(live_crumb.last_log, text, sizeof(live_crumb.last_log) - 1);
    live_crumb.last_log[sizeof(live_crumb.last_log) - 1] = '\0';
    // Stamped here rather than in noteVoltage() so the 500 Hz path never reads the clock.
    live_crumb.uptime_ms = to_ms_since_boot(get_absolute_time());
#else
    (void)text;
#endif
}

void Breadcrumb::report()
{
#ifdef PICO_BUILD
    const uint32_t bits     = vreg_and_chip_reset_hw->chip_reset;
    const bool     had_por  = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) != 0;
    const bool     had_run  = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) != 0;
    const bool     had_psm  = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) != 0;
    const bool     software = watchdog_caused_reboot();

    const Snapshot& prev     = previous_;
    const char*     activity = (prev.last_log[0] != '\0') ? prev.last_log : "(nothing logged yet)";
    char            line[200];

    // POR latches on the first power-up and is never cleared, so it cannot by
    // itself separate a cold boot from a brownout. The surviving snapshot is what
    // separates them: RAM intact means the rail dipped rather than went away.
    if (had_psm)
    {
        LOG_WARNING("Reset cause: debugger restart.");
    }
    else if (software)
    {
        LOG_INFO("Reset cause: software reboot (reflash or bootloader). Not a power fault.");
    }
    else if (had_run)
    {
        LOG_WARNING("Reset cause: external reset (RUN pin or reset button).");
    }
    else if (!prev.valid)
    {
        LOG_INFO("Reset cause: cold power-up. No history from a previous run.");
    }
    else if (prev.voltage_min <= 0.0f)
    {
        std::snprintf(line, sizeof(line), "POWER FAULT: reset after %lu ms, during: %s",
                      static_cast<unsigned long>(prev.uptime_ms), activity);
        LOG_ERROR(line);
        LOG_ERROR("  -> No battery readings were taken, so the rail cannot be assessed.");
    }
    else if (prev.voltage_min < BATTERY_SAG_WARN_V)
    {
        std::snprintf(line, sizeof(line),
                      "POWER FAULT: the battery rail collapsed under load. It fell to %.2f V "
                      "after %lu ms, during: %s",
                      prev.voltage_min, static_cast<unsigned long>(prev.uptime_ms), activity);
        LOG_ERROR(line);
        LOG_ERROR("  -> The pack cannot deliver stall current. Check its charge and health, "
                  "and the motor power wiring.");
    }
    else
    {
        std::snprintf(line, sizeof(line),
                      "POWER FAULT: reset while the rail still read %.2f V, after %lu ms, "
                      "during: %s",
                      prev.voltage_min, static_cast<unsigned long>(prev.uptime_ms), activity);
        LOG_ERROR(line);
        LOG_ERROR("  -> Voltage never sagged, so suspect an intermittent power or ground "
                  "connection rather than the battery.");
    }

    std::snprintf(line, sizeof(line),
                  "chip_reset=0x%08lx por=%d run=%d psm=%d wd=%d crumb=%s uptime_ms=%lu "
                  "v_min=%.3f v_last=%.3f",
                  static_cast<unsigned long>(bits), had_por ? 1 : 0, had_run ? 1 : 0,
                  had_psm ? 1 : 0, software ? 1 : 0, prev.valid ? "valid" : "decayed",
                  static_cast<unsigned long>(prev.uptime_ms), prev.voltage_min, prev.voltage_last);
    LOG_DIAG(line);
#endif
}
