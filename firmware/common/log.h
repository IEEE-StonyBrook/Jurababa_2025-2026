#ifndef COMMON_LOG_H
#define COMMON_LOG_H

#include <iostream>
#include <sstream>
#include <string>

class Bluetooth;

enum class LogPriority
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Log
{
  public:
    static void message(LogPriority priority, std::string msg);
    static void setBluetoothInterface(Bluetooth* bt);
    static void setBluetoothEnabled(bool enabled);
    static bool isBluetoothEnabled();

  private:
    static LogPriority  print_priority_level_;
    static Bluetooth*   bluetooth_;
    static bool         bluetooth_enabled_;
};

#define LOG_DEBUG(msg)                                                         \
    {                                                                          \
        std::ostringstream ss;                                                 \
        ss << msg;                                                             \
        Log::message(LogPriority::DEBUG, ss.str());                            \
    }

#define LOG_INFO(msg)                                                          \
    {                                                                          \
        std::ostringstream ss;                                                 \
        ss << msg;                                                             \
        Log::message(LogPriority::INFO, ss.str());                             \
    }

#define LOG_WARNING(msg)                                                       \
    {                                                                          \
        std::ostringstream ss;                                                 \
        ss << msg;                                                             \
        Log::message(LogPriority::WARN, ss.str());                             \
    }

#define LOG_ERROR(msg)                                                         \
    {                                                                          \
        std::ostringstream ss;                                                 \
        ss << msg;                                                             \
        Log::message(LogPriority::ERROR, ss.str());                            \
    }

#define LOG_FATAL(msg)                                                         \
    {                                                                          \
        std::ostringstream ss;                                                 \
        ss << msg;                                                             \
        Log::message(LogPriority::FATAL, ss.str());                            \
    }

#endif
