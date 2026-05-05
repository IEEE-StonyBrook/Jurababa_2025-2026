#include "common/log.h"

#include <algorithm>
#include <cstring>

#ifdef PICO_BUILD
#include "app/bluetooth.h"
#include "pico/critical_section.h"
#endif

LogPriority Log::print_priority_level_     = LogPriority::DEBUG;
LogPriority Log::bluetooth_priority_level_ = LogPriority::INFO;
Bluetooth*  Log::bluetooth_                = nullptr;
bool        Log::bluetooth_enabled_        = false;

#ifdef PICO_BUILD
namespace
{
constexpr uint16_t kBluetoothLogQueueSize    = 16;
constexpr uint16_t kBluetoothLogMessageBytes = 192;

struct QueuedBluetoothLog
{
    char     text[kBluetoothLogMessageBytes];
    uint16_t length;
};

QueuedBluetoothLog bluetooth_log_queue[kBluetoothLogQueueSize] = {};
critical_section_t bluetooth_log_lock;
bool               bluetooth_log_lock_initialized = false;
volatile uint16_t  bluetooth_log_head             = 0;
volatile uint16_t  bluetooth_log_tail             = 0;
volatile uint16_t  bluetooth_log_count            = 0;
volatile uint16_t  bluetooth_log_max_count        = 0;
volatile uint32_t  bluetooth_log_dropped          = 0;
volatile uint32_t  bluetooth_log_truncated        = 0;

void ensureBluetoothLogQueue()
{
    if (bluetooth_log_lock_initialized)
        return;

    critical_section_init(&bluetooth_log_lock);
    bluetooth_log_lock_initialized = true;
}

void enqueueBluetoothLog(const std::string& msg)
{
    ensureBluetoothLogQueue();

    critical_section_enter_blocking(&bluetooth_log_lock);
    if (bluetooth_log_count >= kBluetoothLogQueueSize)
    {
        ++bluetooth_log_dropped;
        critical_section_exit(&bluetooth_log_lock);
        return;
    }

    QueuedBluetoothLog& slot = bluetooth_log_queue[bluetooth_log_head];
    size_t              len  = msg.size();
    if (len > kBluetoothLogMessageBytes - 3)
    {
        len = kBluetoothLogMessageBytes - 3;
        ++bluetooth_log_truncated;
    }

    std::memcpy(slot.text, msg.data(), len);
    slot.text[len++] = '\r';
    slot.text[len++] = '\n';
    slot.length      = static_cast<uint16_t>(len);

    bluetooth_log_head = static_cast<uint16_t>((bluetooth_log_head + 1) % kBluetoothLogQueueSize);
    ++bluetooth_log_count;
    if (bluetooth_log_count > bluetooth_log_max_count)
        bluetooth_log_max_count = bluetooth_log_count;

    critical_section_exit(&bluetooth_log_lock);
}
} // namespace
#endif

void Log::message(LogPriority priority, std::string msg)
{
    if (static_cast<int>(priority) < static_cast<int>(print_priority_level_))
        return;

    std::string prefix;
    switch (priority)
    {
        case LogPriority::DEBUG:
            prefix = "[DEBUG] ";
            break;
        case LogPriority::INFO:
            prefix = "[INFO] ";
            break;
        case LogPriority::WARN:
            prefix = "[WARN] ";
            break;
        case LogPriority::ERROR:
            prefix = "[ERROR] ";
            break;
        case LogPriority::FATAL:
            prefix = "[FATAL] ";
            break;
    }

    std::string final_msg = prefix + msg;

#ifdef SIMULATOR_BUILD
    // Use stderr for simulator to avoid interfering with mms protocol on stdout
    std::cerr << final_msg << '\n';
#else
    std::cout << final_msg << '\n';
#endif

#ifdef PICO_BUILD
    if (bluetooth_enabled_ && bluetooth_ != nullptr &&
        static_cast<int>(priority) >= static_cast<int>(bluetooth_priority_level_))
    {
        enqueueBluetoothLog(final_msg);
    }
#endif
}

void Log::setBluetoothInterface(Bluetooth* bt)
{
#ifdef PICO_BUILD
    ensureBluetoothLogQueue();
#endif
    bluetooth_ = bt;
}

void Log::setBluetoothEnabled(bool enabled)
{
    bluetooth_enabled_ = enabled;
}

void Log::setBluetoothPriority(LogPriority priority)
{
    bluetooth_priority_level_ = priority;
}

void Log::drainBluetooth()
{
#ifdef PICO_BUILD
    if (!bluetooth_enabled_ || bluetooth_ == nullptr)
        return;

    ensureBluetoothLogQueue();

    while (true)
    {
        char     text[kBluetoothLogMessageBytes];
        uint16_t length = 0;

        critical_section_enter_blocking(&bluetooth_log_lock);
        if (bluetooth_log_count == 0)
        {
            critical_section_exit(&bluetooth_log_lock);
            break;
        }

        QueuedBluetoothLog& slot = bluetooth_log_queue[bluetooth_log_tail];
        length                   = slot.length;
        std::memcpy(text, slot.text, length);

        bluetooth_log_tail =
            static_cast<uint16_t>((bluetooth_log_tail + 1) % kBluetoothLogQueueSize);
        --bluetooth_log_count;
        critical_section_exit(&bluetooth_log_lock);

        bluetooth_->writeBytes(reinterpret_cast<const uint8_t*>(text), length);
    }
#endif
}

bool Log::isBluetoothEnabled()
{
    return bluetooth_enabled_ && (bluetooth_ != nullptr);
}

Log::BluetoothDiagnostics Log::bluetoothDiagnostics()
{
#ifdef PICO_BUILD
    ensureBluetoothLogQueue();

    critical_section_enter_blocking(&bluetooth_log_lock);
    BluetoothDiagnostics diag = {bluetooth_log_count, bluetooth_log_max_count,
                                 bluetooth_log_dropped, bluetooth_log_truncated};
    critical_section_exit(&bluetooth_log_lock);
    return diag;
#else
    return {0, 0, 0, 0};
#endif
}
