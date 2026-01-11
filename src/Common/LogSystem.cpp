#include "Common/LogSystem.h"

// Only include BluetoothInterface on Pico platform
#ifdef PICO_BUILD
#include "Platform/Pico/BluetoothInterface.h"
#endif

// Static member initialization
LogPriority         LogSystem::printPriorityLevel   = LogPriority::INFO;
BluetoothInterface* LogSystem::bluetooth_interface_ = nullptr;
bool                LogSystem::bluetooth_enabled_   = false;

void LogSystem::logMessage(LogPriority logPriority, std::string logMessage)
{
    if (static_cast<int>(logPriority) < static_cast<int>(printPriorityLevel))
        return;

    std::string prefixForLogMessage;
    switch (logPriority)
    {
        case LogPriority::DEBUG:
            prefixForLogMessage = "[DEBUG] ";
            break;
        case LogPriority::INFO:
            prefixForLogMessage = "[INFO] ";
            break;
        case LogPriority::WARN:
            prefixForLogMessage = "[WARN] ";
            break;
        case LogPriority::ERROR:
            prefixForLogMessage = "[ERROR] ";
            break;
        case LogPriority::FATAL:
            prefixForLogMessage = "[FATAL] ";
            break;
    }

    std::string finalMessage = prefixForLogMessage + logMessage;

    // Console log (USB serial or stdout)
    std::cout << finalMessage << '\n';

    // Bluetooth log (if enabled and interface is set)
#ifdef PICO_BUILD
    if (bluetooth_enabled_ && bluetooth_interface_ != nullptr)
    {
        bluetooth_interface_->write(finalMessage + "\r\n");
    }
#endif
}

void LogSystem::setBluetoothInterface(BluetoothInterface* bt)
{
    bluetooth_interface_ = bt;
}

void LogSystem::setBluetoothEnabled(bool enabled)
{
    bluetooth_enabled_ = enabled;
}

bool LogSystem::isBluetoothEnabled()
{
    return bluetooth_enabled_ && (bluetooth_interface_ != nullptr);
}