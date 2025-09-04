#include "PathConverter.h"

PathConverter::PathConverter(API* api, InternalMouse* internalMouse, LogSystem* logSystem,
                             bool diagMovementAllowed)
    : api(api),
      internalMouse(internalMouse),
      logSystem(logSystem),
      diagMovementAllowed(diagMovementAllowed) {}

std::string PathConverter::buildLFRPath(std::vector<MazeNode*> nodePath) {
  std::string lfrPath;
  if (nodePath.size() < 2) return lfrPath;

  std::array<int, 2> currHeading = internalMouse->getCurrentRobotDir();
  MazeNode* currNode = internalMouse->getCurrentRobotNode();
  MazeNode* nextNode = nodePath[0];

  
}