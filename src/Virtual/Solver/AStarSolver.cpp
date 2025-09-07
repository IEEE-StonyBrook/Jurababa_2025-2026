#include "AStarSolver.h"

AStarSolver::AStarSolver(API* api, InternalMouse* internalMouse)
    : api(api), internalMouse(internalMouse) {}

std::string AStarSolver::go(std::vector<std::array<int, 2>> endCells,
                            bool diagMovementAllowed,
                            bool passThroughGoalCells) {
  std::vector<MazeNode*> aStarPath =
      getBestPathToEndCell(endCells, diagMovementAllowed, passThroughGoalCells);
  std::string stringPath = getStringPath(aStarPath);
  LOG_DEBUG(stringPath);
  std::array<int, 2> c = internalMouse->getCurrentRobotDirArray();
  std::string lfrPath = PathConverter::buildLFRPath(
     internalMouse->getCurrentRobotNode(),
     internalMouse->getCurrentRobotDirArray(), aStarPath);
  return lfrPath;
}

std::vector<MazeNode*> AStarSolver::getBestPathToEndCell(
    std::vector<std::array<int, 2>> endCells, bool diagMovementAllowed,
    bool passThroughGoalCells) {
  std::vector<MazeNode*> bestPath;
  float bestPathCost = std::numeric_limits<float>::infinity();

  for (std::array<int, 2> endCell : endCells) {
    std::vector<MazeNode*> pathToEnd = getPathFromCurrPosToCell(
        internalMouse->getNodeAtPos(endCell[0], endCell[1]),
        diagMovementAllowed, passThroughGoalCells);
    if (pathToEnd.empty()) continue;
    if (totalPathCost < bestPathCost) {
      bestPath = pathToEnd;
      bestPathCost = totalPathCost;
    }
  }

  return bestPath;
}

struct AStarNode {
  MazeNode* node;
  float sCost, fCost;

  // Used for the priority queue in assigning which cells are first.
  // E.g. A has lower priority than B (A < B) so A.fCost > B.fCost.
  bool operator<(const AStarNode& other) const { return fCost > other.fCost; }
};

std::vector<MazeNode*> AStarSolver::getPathFromCurrPosToCell(
    MazeNode* endCell, bool diagMovementAllowed, bool passThroughGoalCells) {
  MazeNode* startNode = internalMouse->getCurrentRobotNode();

  std::vector<std::vector<float>> sCostOfCells(
      internalMouse->getMazeWidth(),
      std::vector<float>(internalMouse->getMazeHeight(),
                         std::numeric_limits<float>::infinity()));

  std::priority_queue<AStarNode> toBeProcessedCells;

  sCostOfCells[startNode->getCellXPos()][startNode->getCellYPos()] = 0.0f;
  toBeProcessedCells.push(
      {startNode, 0.0f, getHeuristicDistance(startNode, endCell)});

  while (!toBeProcessedCells.empty()) {
    AStarNode currentNode = toBeProcessedCells.top();
    toBeProcessedCells.pop();

    // Re-construct A* path if end is reached.
    if (currentNode.node == endCell) {
      totalPathCost = currentNode.fCost;
      return constructPath(startNode, endCell);
    }

    // If cell is already processed, skip; otherwise, mark as so.
    if (currentNode.node->isProcessed) continue;
    currentNode.node->isProcessed = true;

    for (MazeNode* neighbor : internalMouse->getNodeNeighbors(
             currentNode.node, diagMovementAllowed)) {
      // Continue if neighbor is already processed,
      // is a goal cell (if enabled), or is unreachable.
      bool c = internalMouse->isAGoalCell(neighbor);
      bool d = !internalMouse->getCanMoveBetweenNodes(
          currentNode.node, neighbor, diagMovementAllowed);
      if (neighbor->isProcessed ||
          (!passThroughGoalCells && internalMouse->isAGoalCell(neighbor)) ||
          !internalMouse->getCanMoveBetweenNodes(currentNode.node, neighbor,
                                                 diagMovementAllowed)) {
        continue;
      }

      // Euclidean distance = (1 or sqrt(2)) for a neighbor.
      float newS =
          currentNode.sCost + getHeuristicDistance(currentNode.node, neighbor);

      int neighborX = neighbor->getCellXPos();
      int neighborY = neighbor->getCellYPos();
      // Updates start->cell (sCost) cost if it's less than the stored cost.
      if (newS < sCostOfCells[neighborX][neighborY]) {
        sCostOfCells[neighborX][neighborY] = newS;
        neighbor->parentNode = currentNode.node;
        toBeProcessedCells.push(
            {neighbor, newS, newS + getHeuristicDistance(neighbor, endCell)});
      }
    }
  }

  LOG_ERROR("AStarSolver.cpp: No path was produced!")
  return {};
}

float AStarSolver::getHeuristicDistance(MazeNode* from, MazeNode* to) {
  const std::string HEURISTIC_USED = "octile";

  float diffX = fabs(from->getCellXPos() - to->getCellXPos());
  float diffY = fabs(from->getCellYPos() - to->getCellYPos());

  if (HEURISTIC_USED == "octile") {
    return (diffX + diffY) + (std::sqrt(2) - 2) * fmin(diffX, diffY);
  } else if (HEURISTIC_USED == "euclidean") {
    return std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2));
  } else if (HEURISTIC_USED == "manhattan") {
    return diffX + diffY;
  }

  return 0.0f;
}

std::vector<MazeNode*> AStarSolver::constructPath(MazeNode* startNode,
                                                  MazeNode* endNode) {
  std::vector<MazeNode*> path;
  MazeNode* nodePtr = endNode;

  while (nodePtr && nodePtr != startNode) {
    path.push_back(nodePtr);
    nodePtr = nodePtr->parentNode;
  }

  std::reverse(path.begin(), path.end());

  // Resets all previous cells and processed markings.
  internalMouse->resetSolverVariables();
  return path;
}

std::string AStarSolver::getStringPath(std::vector<MazeNode*> path) {
  std::string pathString;

  for (int i = 0; i < path.size(); i++) {
    pathString += "(" + std::to_string(path[i]->getCellXPos()) + "," +
                  std::to_string(path[i]->getCellYPos()) + ")";
    if (i < path.size() - 1) pathString += " -> ";
  }
  return pathString;
}