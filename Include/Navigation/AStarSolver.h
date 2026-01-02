#ifndef ASTARSOLVER_H
#define ASTARSOLVER_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "Common/LogSystem.h"
#include "Navigation/PathConverter.h"
#include "Navigation/Diagonalizer.h"

class AStarSolver
{
  public:
    AStarSolver(InternalMouse* internalMouse);

    std::string go(std::vector<std::array<int, 2>> endCells, bool diagMovementAllowed = false,
                   bool passThroughGoalCells = false);

    /**
     * @brief Gets the cell-based path to the goal cells.
     * @return Vector of MazeNode pointers representing the path cells.
     */
    std::vector<MazeNode*> getCellPath(std::vector<std::array<int, 2>> endCells,
                                       bool diagMovementAllowed = false,
                                       bool passThroughGoalCells = false);

  private:
    std::vector<MazeNode*> getBestPathToEndCell(std::vector<std::array<int, 2>> endCells,
                                                bool                            diagMovementAllowed,
                                                bool passThroughGoalCells);
    std::vector<MazeNode*> getPathFromCurrPosToCell(MazeNode* endCell, bool diagMovementAllowed,
                                                    bool passThroughGoalCells);

    std::vector<MazeNode*> constructPath(MazeNode* startNode, MazeNode* endNode);
    static std::string     getStringPath(std::vector<MazeNode*> path);
    static float           getHeuristicDistance(MazeNode* from, MazeNode* to);

    InternalMouse* internalMouse;

    float totalPathCost;
};

#endif