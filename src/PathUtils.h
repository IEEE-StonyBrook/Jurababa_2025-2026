#ifndef PATHUTILS_H
#define PATHUTILS_H

#include "../Include/Maze/InternalMouse.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Platform/IAPIInterface.h"

void interpretLFRPath(IAPIInterface* apiPtr, std::string lfrPath);
bool traversePathIteratively(IAPIInterface* apiPtr, InternalMouse* mouse,
                             std::vector<std::array<int, 2>> goalCells, bool diagonalsAllowed,
                             bool allExplored, bool avoidGoalCells);
void setAllExplored(InternalMouse* mouse);
void detectWalls(IAPIInterface& api, InternalMouse& internalMouse);

#endif
