#include "common/log.h"

#ifdef PICO_BUILD
#include "app/bluetooth.h"
#endif

LogPriority Log::print_priority_level_ = LogPriority::INFO;
Bluetooth*  Log::bluetooth_            = nullptr;
bool        Log::bluetooth_enabled_    = false;

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
    if (bluetooth_enabled_ && bluetooth_ != nullptr)
    {
        bluetooth_->write(final_msg + "\r\n");
    }
#endif
}

void Log::setBluetoothInterface(Bluetooth* bt)
{
    bluetooth_ = bt;
}

void Log::setBluetoothEnabled(bool enabled)
{
    bluetooth_enabled_ = enabled;
}

bool Log::isBluetoothEnabled()
{
    return bluetooth_enabled_ && (bluetooth_ != nullptr);
}
