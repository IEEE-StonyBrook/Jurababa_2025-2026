#ifndef FRONTIERBASED_H
#define FRONTIERBASED_H

#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Common/LogSystem.h"
#include "Maze/InternalMouse.h"
#include "Maze/MazeGraph.h"
#include "Platform/IAPIInterface.h"

/**
 * @class FrontierBased
 * @brief Implements the Frontier-Based exploration algorithm.
 *
 * This class provides exploration logic for the micromouse by
 * expanding frontiers (unexplored nodes) until all reachable
 * nodes are visited. It uses BFS distance to select the closest
 * frontier and traverses using A*.
 */
class FrontierBased {
 public:
  /**
   * @brief Explores the maze using the Frontier-Based algorithm.
   *
   * @param mouse Reference to the InternalMouse instance representing the
   * mouse's state.
   * @param api Reference to the IAPIInterface instance for interacting with the maze
   * display (platform-agnostic).
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   */
  static void explore(InternalMouse& mouse, IAPIInterface& api, bool diagonalsAllowed);

 private:
  /**
   * @brief Picks the closest frontier using BFS to find distances from the
   * current node.
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param frontiers Reference to the set of frontier nodes.
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   * @return MazeNode* Pointer to the closest frontier node, or nullptr if none
   * is reachable.
   */
  static MazeNode* pickNextFrontier(InternalMouse& mouse,
                                    std::unordered_set<MazeNode*>& frontiers,
                                    bool diagonalsAllowed);

  /**
   * @brief Performs BFS to calculate distances from the start node to all
   * reachable nodes.
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param startNode Pointer to the starting MazeNode.
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   * @return std::unordered_map<MazeNode*, double> Map of nodes to their
   * distance from the start node.
   */
  static std::unordered_map<MazeNode*, double> getBFSDist(
      InternalMouse& mouse, MazeNode* startNode, bool diagonalsAllowed);
};

#endif  // FRONTIERBASED_H