#include "../../Include/Navigation/FrontierBasedSearchSolver.h"

#include <queue>
#include <limits>
#include <cmath>

FrontierBasedSearchSolver::FrontierBasedSearchSolver(
    API* api, InternalMouse* internalMouse, bool diagMovementAllowed)
    : api(api),
      internalMouse(internalMouse),
      diagMovementAllowed(diagMovementAllowed) {}

void FrontierBasedSearchSolver::exploreMaze() {
  MazeNode* startNode = internalMouse->getCurrentRobotNode();
  frontierNodes.insert(startNode);
  startNode->markAsExplored();

  while (!frontierNodes.empty()) {
    MazeNode* nextFrontier = getOptimalFrontierToExploreNext();
    if (!nextFrontier) break;

    // update robot’s position
    internalMouse->setCurrentPosition(nextFrontier);

    // mark as explored
    nextFrontier->markAsExplored();
    api->setText(nextFrontier->getCellXPos(), nextFrontier->getCellYPos(), "");
    frontierNodes.erase(nextFrontier);

    // add valid unexplored neighbors
    for (MazeNode* neighbor :
         internalMouse->getNodeNeighbors(nextFrontier, diagMovementAllowed)) {
      if (!neighbor->getCellIsExplored() &&
          internalMouse->getCanMoveBetweenNodes(nextFrontier, neighbor,
                                                diagMovementAllowed) &&
          !internalMouse->isAGoalCell(neighbor)) {
        api->setText(neighbor->getCellXPos(), neighbor->getCellYPos(), "*");
        frontierNodes.insert(neighbor);
      }
    }
  }

  // sweep goal cells last
  for (auto goal : internalMouse->getGoalCells()) {
    MazeNode* goalNode = internalMouse->getNodeAtPos(goal[0], goal[1]);
    internalMouse->setCurrentPosition(goalNode);
  }
}

MazeNode* FrontierBasedSearchSolver::getOptimalFrontierToExploreNext() {
  MazeNode* curr = internalMouse->getCurrentRobotNode();
  MazeNode* bestNode = nullptr;
  float bestDist = std::numeric_limits<float>::infinity();

  auto distMap = getBFSCosts(curr);

  for (MazeNode* frontier : frontierNodes) {
    auto it = distMap.find(frontier);
    if (it != distMap.end()) {
      float d = it->second;
      if (d < bestDist) {
        bestDist = d;
        bestNode = frontier;
      }
    }
  }
  return bestNode;
}

std::unordered_map<MazeNode*, float>
FrontierBasedSearchSolver::getBFSCosts(MazeNode* startBFSCell) {
  std::unordered_map<MazeNode*, float> distMap;
  std::queue<MazeNode*> q;

  distMap[startBFSCell] = 0.0f;
  q.push(startBFSCell);

  while (!q.empty()) {
    MazeNode* curr = q.front();
    q.pop();
    float currDist = distMap[curr];

    for (MazeNode* neighbor :
         internalMouse->getNodeNeighbors(curr, diagMovementAllowed)) {
      if (distMap.find(neighbor) == distMap.end() &&
          internalMouse->getCanMoveBetweenNodes(curr, neighbor,
                                                diagMovementAllowed)) {
        distMap[neighbor] =
            currDist +
            std::sqrt(std::pow(curr->getCellXPos() - neighbor->getCellXPos(), 2) +
                      std::pow(curr->getCellYPos() - neighbor->getCellYPos(), 2));
        q.push(neighbor);
      }
    }
  }
  return distMap;
}