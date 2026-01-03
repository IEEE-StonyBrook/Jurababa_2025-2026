#include "Navigation/FrontierBasedSearchSolver.h"

#include "Navigation/PathUtils.h"

/**
 * @brief Explores the maze using the Frontier-Based algorithm.
 *
 * @param mouse Reference to the InternalMouse instance representing the mouse's
 * state.
 * @param api Reference to the IAPIInterface instance for interacting with the maze
 * display (platform-agnostic).
 * @param diagonalsAllowed Whether diagonal movements are permitted.
 */
void FrontierBased::explore(InternalMouse& mouse, IAPIInterface& api,
                            bool diagonalsAllowed) {
  std::unordered_set<MazeNode*> frontiers;
  MazeNode* start = mouse.getCurrentRobotNode();
  frontiers.insert(start);
  start->markAsExplored();
  MazeNode* currNode = start;

  while (!frontiers.empty()) {
    // LOG_INFO("Found new frontier");

    // 1) Pick the closest frontier.
    MazeNode* nextFrontier =
        pickNextFrontier(mouse, frontiers, diagonalsAllowed);
    if (nextFrontier == nullptr) {
      break;
    }
    // LOG_DEBUG("Next frontier at (" +
    //           std::to_string(nextFrontier->getCellXPos()) + "," +
    //           std::to_string(nextFrontier->getCellYPos()) + ")");

    // LOG_INFO("Continuing explore");
    bool moved = traversePathIteratively(
        &api, &mouse,
        {{nextFrontier->getCellXPos(), nextFrontier->getCellYPos()}},
        diagonalsAllowed, false, true);
    // LOG_INFO("Traversed");
    if (!moved) {
      api.setText(nextFrontier->getCellXPos(), nextFrontier->getCellYPos(), "");
      frontiers.erase(nextFrontier);
      // continue;
    }

    // 3) Arrived: detect walls, mark as explored.
    currNode = mouse.getCurrentRobotNode();
    detectWalls(api, mouse);
    currNode->markAsExplored();
    api.setText(currNode->getCellXPos(), currNode->getCellYPos(), "");
    frontiers.erase(currNode);

    // 4) Optimization: Mark neighbors with 3+ walls as explored (dead ends).
    // A cell with 3 walls has only one entrance, so once we know about it,
    // there's no need to physically visit it.
    std::vector<MazeNode*> allNeighbors =
        mouse.getNodeNeighbors(currNode, false);  // Cardinal only for wall check
    for (MazeNode* neighbor : allNeighbors) {
      if (!neighbor->getCellIsExplored() && neighbor->getWallCount() >= 3) {
        neighbor->markAsExplored();
        api.setText(neighbor->getCellXPos(), neighbor->getCellYPos(), "");
        frontiers.erase(neighbor);
      }
    }

    // 5) Add valid neighbors as frontiers (ignore goal cells).
    std::vector<MazeNode*> neighbors =
        mouse.getNodeNeighbors(currNode, diagonalsAllowed);
    for (MazeNode* neighbor : neighbors) {
      if (!neighbor->getCellIsExplored() &&
          mouse.getCanMoveBetweenNodes(currNode, neighbor, diagonalsAllowed) &&
          !mouse.isAGoalCell(neighbor)) {
        api.setText(neighbor->getCellXPos(), neighbor->getCellYPos(), "*");
        frontiers.insert(neighbor);
      }
    }
  }

  // 5) Finally, visit each avoided goal cell (if reachable).
  traversePathIteratively(&api, &mouse, mouse.getGoalCells(), diagonalsAllowed,
                          false, false);
}

/**
 * @brief Picks the closest frontier using BFS to find distances from current
 * node.
 */
MazeNode* FrontierBased::pickNextFrontier(
    InternalMouse& mouse, std::unordered_set<MazeNode*>& frontiers,
    bool diagonalsAllowed) {
  MazeNode* currNode = mouse.getCurrentRobotNode();
  MazeNode* bestNode = nullptr;
  double bestDist = std::numeric_limits<double>::infinity();

  // Get BFS distances from current node to all reachable nodes
  std::unordered_map<MazeNode*, double> distMap =
      getBFSDist(mouse, currNode, diagonalsAllowed);

  for (MazeNode* frontier : frontiers) {
    auto it = distMap.find(frontier);
    if (it != distMap.end()) {
      double dist = it->second;
      if (dist < bestDist) {
        bestDist = dist;
        bestNode = frontier;
      }
    }
  }
  return bestNode;
}

/**
 * @brief Performs BFS to calculate distances from the start node to all
 * reachable nodes.
 */
std::unordered_map<MazeNode*, double> FrontierBased::getBFSDist(
    InternalMouse& mouse, MazeNode* startNode, bool diagonalsAllowed) {
  std::queue<MazeNode*> queue;
  std::unordered_map<MazeNode*, double> distMap;

  distMap[startNode] = 0.0;
  queue.push(startNode);

  while (!queue.empty()) {
    MazeNode* currNode = queue.front();
    queue.pop();
    double currDist = distMap[currNode];

    std::vector<MazeNode*> neighbors =
        mouse.getNodeNeighbors(currNode, diagonalsAllowed);
    for (MazeNode* neighbor : neighbors) {
      if (distMap.find(neighbor) == distMap.end() &&
          mouse.getCanMoveBetweenNodes(currNode, neighbor, diagonalsAllowed)) {
        distMap[neighbor] =
            currDist + InternalMouse::euclideanDistance(*currNode, *neighbor);
        queue.push(neighbor);
      }
    }
  }
  return distMap;
}