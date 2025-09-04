#include "LogSystem.h"

#include <iostream>

LogPriority LogSystem::printPriorityLevel = LogPriority::DEBUG;

void LogSystem::logMessage(LogPriority logPriority, std::string logMessage) {
  if (static_cast<int>(logPriority) < static_cast<int>(printPriorityLevel))
    return;

  std::string prefixForLogMessage;
  switch (logPriority) {
    case LogPriority::DEBUG:
      prefixForLogMessage = "[DEBUG] ";
      break;
    case LogPriority::INFO:
      prefixForLogMessage = "[INFO] ";
      break;
    case LogPriority::WARN:
      prefixForLogMessage = "[WARN]";
      break;
    case LogPriority::ERROR:
      prefixForLogMessage = "[ERROR]";
      break;
    case LogPriority::FATAL: 
      prefixForLogMessage = "[FATAL]";
      break;
  }
  std::cout << prefixForLogMessage << logMessage << '\n';
}