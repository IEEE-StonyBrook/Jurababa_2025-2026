#ifndef LOGSYSTEM_H
#define LOGSYSTEM_H

#include <sstream>
#include <string>

// Forward declaration (to avoid circular include)
class Bluetooth;

enum class LogPriority { DEBUG, INFO, WARN, ERROR, FATAL };

class LogSystem {
 public:
  static void logMessage(LogPriority logPriority, std::string logMessage);
  static void attachBluetooth(Bluetooth* bt);  // NEW

 private:
  static LogPriority printPriorityLevel;
  static Bluetooth* btInterface;  // NEW
};

// ---------------- Macros ----------------
#define LOG_DEBUG(logMsg)                                             \
  {                                                                   \
    std::ostringstream logStringStream;                               \
    logStringStream << logMsg;                                        \
    LogSystem::logMessage(LogPriority::DEBUG, logStringStream.str()); \
  }

#define LOG_INFO(logMsg)                                             \
  {                                                                  \
    std::ostringstream logStringStream;                              \
    logStringStream << logMsg;                                       \
    LogSystem::logMessage(LogPriority::INFO, logStringStream.str()); \
  }

#define LOG_WARNING(logMsg)                                          \
  {                                                                  \
    std::ostringstream logStringStream;                              \
    logStringStream << logMsg;                                       \
    LogSystem::logMessage(LogPriority::WARN, logStringStream.str()); \
  }

#define LOG_ERROR(logMsg)                                             \
  {                                                                   \
    std::ostringstream logStringStream;                               \
    logStringStream << logMsg;                                        \
    LogSystem::logMessage(LogPriority::ERROR, logStringStream.str()); \
  }

#define LOG_FATAL(logMsg)                                             \
  {                                                                   \
    std::ostringstream logStringStream;                               \
    logStringStream << logMsg;                                        \
    LogSystem::logMessage(LogPriority::FATAL, logStringStream.str()); \
  }

#endif