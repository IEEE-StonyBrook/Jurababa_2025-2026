#ifndef FRONTIER_BASED_SEARCH_SOLVER_H
#define FRONTIER_BASED_SEARCH_SOLVER_H

#include <unordered_set>
#include <unordered_map>
#include "../Platform/Simulator/API.h"
#include "../Maze/MazeNode.h"
#include "../Maze/InternalMouse.h"

class FrontierBasedSearchSolver {
 public:
  FrontierBasedSearchSolver(API* api, InternalMouse* internalMouse,
                            bool diagMovementAllowed);

  void exploreMaze();

 private:
  API* api;
  InternalMouse* internalMouse;
  bool diagMovementAllowed;

  std::unordered_set<MazeNode*> frontierNodes;

  MazeNode* getOptimalFrontierToExploreNext();
  std::unordered_map<MazeNode*, float> getBFSCosts(MazeNode* startBFSCell);
};

#endif  // FRONTIER_BASED_SEARCH_SOLVER_H