#ifndef PATHUTILS_H
#define PATHUTILS_H

#include "Maze/InternalMouse.h"
#include "Navigation/AStarSolver.h"
#include "Platform/IAPIInterface.h"

void interpretLFRPath(IAPIInterface* apiPtr, std::string lfrPath);
bool traversePathIteratively(IAPIInterface* apiPtr, InternalMouse* mouse,
                             std::vector<std::array<int, 2>> goalCells, bool diagonalsAllowed,
                             bool allExplored, bool avoidGoalCells);
void setAllExplored(InternalMouse* mouse);
void detectWalls(IAPIInterface& api, InternalMouse& internalMouse);

/**
 * @brief Colors the cells in the given path with the current phase color.
 * @param apiPtr Pointer to the API interface.
 * @param cellPath Vector of MazeNode pointers representing the path cells.
 */
void colorPathCells(IAPIInterface* apiPtr, const std::vector<MazeNode*>& cellPath);

#endif
