#ifndef FRONTIERBASEDSEARCHSOLVER_H
#define FRONTIERBASEDSEARCHSOLVER_H

#include <unordered_map>
#include <unordered_set>

#include "../Maze/InternalMouse.h"
#include "../Platform/Simulator/API.h"

class FrontierBasedSearchSolver
{
  public:
    FrontierBasedSearchSolver(API* api, InternalMouse* internalMouse,
                              bool diagMovementAllowed = false);
    ~FrontierBasedSearchSolver();

    void exploreMaze();

  private:
    MazeNode*                            getOptimalFrontierToExploreNext();
    std::unordered_map<MazeNode*, float> getBFSCosts(MazeNode* startBFSCell);

    API*           api;
    InternalMouse* internalMouse;
    bool           diagMovementAllowed;

    std::unordered_set<MazeNode*> frontierNodes;
};

#endif