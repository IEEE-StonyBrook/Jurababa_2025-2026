#ifndef LOGSYSTEM_H
#define LOGSYSTEM_H

#include <iostream>
#include <sstream>
#include <string>

// Forward declaration to avoid circular includes
class BluetoothInterface;

enum class LogPriority
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class LogSystem
{
  public:
    /**
     * @brief Logs a message to console (and Bluetooth if enabled).
     *
     * @param logPriority Priority level of the message.
     * @param logMessage The message to log.
     */
    static void logMessage(LogPriority logPriority, std::string logMessage);

    /**
     * @brief Sets the Bluetooth interface for wireless logging.
     *
     * @param bt Pointer to BluetoothInterface instance (nullptr to disable).
     */
    static void setBluetoothInterface(BluetoothInterface* bt);

    /**
     * @brief Enables or disables Bluetooth logging.
     *
     * @param enabled True to enable Bluetooth output, false to disable.
     */
    static void setBluetoothEnabled(bool enabled);

    /**
     * @brief Checks if Bluetooth logging is currently enabled.
     *
     * @return true if Bluetooth logging is enabled and interface is set.
     */
    static bool isBluetoothEnabled();

  private:
    static LogPriority         printPriorityLevel;
    static BluetoothInterface* bluetooth_interface_;
    static bool                bluetooth_enabled_;
};

// ---------------- Macros ----------------
#define LOG_DEBUG(logMsg)                                                                          \
    {                                                                                              \
        std::ostringstream logStringStream;                                                        \
        logStringStream << logMsg;                                                                 \
        LogSystem::logMessage(LogPriority::DEBUG, logStringStream.str());                          \
    }

#define LOG_INFO(logMsg)                                                                           \
    {                                                                                              \
        std::ostringstream logStringStream;                                                        \
        logStringStream << logMsg;                                                                 \
        LogSystem::logMessage(LogPriority::INFO, logStringStream.str());                           \
    }

#define LOG_WARNING(logMsg)                                                                        \
    {                                                                                              \
        std::ostringstream logStringStream;                                                        \
        logStringStream << logMsg;                                                                 \
        LogSystem::logMessage(LogPriority::WARN, logStringStream.str());                           \
    }

#define LOG_ERROR(logMsg)                                                                          \
    {                                                                                              \
        std::ostringstream logStringStream;                                                        \
        logStringStream << logMsg;                                                                 \
        LogSystem::logMessage(LogPriority::ERROR, logStringStream.str());                          \
    }

#define LOG_FATAL(logMsg)                                                                          \
    {                                                                                              \
        std::ostringstream logStringStream;                                                        \
        logStringStream << logMsg;                                                                 \
        LogSystem::logMessage(LogPriority::FATAL, logStringStream.str());                          \
    }

#endif