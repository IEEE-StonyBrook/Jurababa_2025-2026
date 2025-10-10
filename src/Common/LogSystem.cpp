#include "../../Include/Common/LogSystem.h"

#include <iostream>

#include "../../Include/Common/Bluetooth.h"


LogPriority LogSystem::printPriorityLevel = LogPriority::DEBUG;
Bluetooth* LogSystem::btInterface = nullptr;

void LogSystem::attachBluetooth(Bluetooth* bt) { btInterface = bt; }

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

  // Console log (original)
  std::cout << finalMessage << '\n';

  // Bluetooth log (new)
  if (btInterface) {
    btInterface->sendString(finalMessage + "\r\n");
  }
}