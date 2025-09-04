#ifndef ASTARSOLVER_H
#define ASTARSOLVER_H

#include "../../API.h"

class AStarSolver {
 public:
  AStarSolver(API* api, InternalMouse* internalMouse);

  std::string go(std::vector<std::array<int, 2>> endCells,
                 bool diagMovementAllowed = false,
                 bool passThroughGoalCells = false);

 private:
  std::vector<MazeNode*> getBestPathToEndCell(
      std::vector<std::array<int, 2>> endCells, bool diagMovementAllowed,
      bool passThroughGoalCells);
  std::vector<MazeNode*> getPathFromCurrPosToCell(MazeNode* endCell,
                                                  bool diagMovementAllowed,
                                                  bool passThroughGoalCells);

  std::vector<MazeNode*> constructPath(MazeNode* startNode,
                                              MazeNode* endNode);
  static std::string getStringPath(std::vector<MazeNode*> path);
  static float getHeuristicDistance(MazeNode* from, MazeNode* to);

  API* api;
  InternalMouse* internalMouse;

  float totalPathCost;
};

#endif