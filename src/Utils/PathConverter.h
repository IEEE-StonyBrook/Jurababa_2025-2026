#ifndef PATHCONVERTER_H
#define PATHCONVERTER_H

#include "../../API.h"
#include "../../Utils/LogSystem.h"
#include "../Mouse/InternalMouse.h"

class PathConverter {
 public:
  PathConverter(API* api, InternalMouse* internalMouse, LogSystem* logSystem,
                bool diagMovementAllowed = false);
  ~PathConverter();

  std::string buildLFRPath(std::vector<MazeNode*> nodePath);

 private:
  API* api;
  InternalMouse* internalMouse;
  LogSystem* logSystem;
  bool diagMovementAllowed;
};

#endif