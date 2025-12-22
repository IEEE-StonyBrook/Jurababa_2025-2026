#ifndef PATHUTILS_H
#define PATHUTILS_H

#include "../Include/Maze/InternalMouse.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Platform/Simulator/API.h"

void interpretLFRPath(API_SIMULATOR* apiPtr, std::string lfrPath);
bool traversePathIteratively(API_SIMULATOR* apiPtr, InternalMouse* mouse,
                             std::vector<std::array<int, 2>> goalCells, bool diagonalsAllowed,
                             bool allExplored, bool avoidGoalCells);
void setAllExplored(InternalMouse* mouse);
void detectWalls(API_SIMULATOR& api, InternalMouse& internalMouse);

#endif
