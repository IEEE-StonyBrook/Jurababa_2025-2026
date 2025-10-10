#include "../../Include/Navigation/FrontierBasedSearchSolver.h"

#include <queue>

FrontierBasedSearchSolver::FrontierBasedSearchSolver(API* api, InternalMouse* internalMouse,
                                                     bool diagMovementAllowed)
    : api(api), internalMouse(internalMouse), diagMovementAllowed(diagMovementAllowed)
{
}

void FrontierBasedSearchSolver::exploreMaze()
{
    MazeNode* startNode = internalMouse->getCurrentRobotNode();
    frontierNodes.insert(startNode);
    startNode->markAsExplored();

    while (!frontierNodes.empty())
    {
        MazeNode* nextFrontier = getOptimalFrontierToExploreNext();
        if (!nextFrontier)
            break;
    }
}

MazeNode* FrontierBasedSearchSolver::getOptimalFrontierToExploreNext()
{
    return nullptr;
}

std::unordered_map<MazeNode*, float> FrontierBasedSearchSolver::getBFSCosts(MazeNode* startBFSCell)
{
    std::unordered_map<MazeNode*, float> distancesToOtherCells = {{startBFSCell, 0.0f}};
    std::queue<MazeNode*>                bfsQueue              = {};

    while (!bfsQueue.empty())
    {
        MazeNode* currentCell = bfsQueue.front();
        bfsQueue.pop();

        for (MazeNode* neighbor : internalMouse->getNodeNeighbors(currentCell))
        {
        }
    }
}