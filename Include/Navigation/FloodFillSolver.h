#ifndef FLOODFILLSOLVER_H
#define FLOODFILLSOLVER_H

#include <queue>
#include <vector>

#include "Common/LogSystem.h"
#include "Maze/InternalMouse.h"
#include "Maze/MazeGraph.h"
#include "Platform/IAPIInterface.h"
#include "Platform/Pico/Config.h"

/**
 * @class FloodFillSolver
 * @brief Implements Flood-Fill maze exploration algorithm.
 *
 * This class provides exploration logic for the micromouse using
 * flood-fill with distance gradient. This approach:
 * - Maintains a distance grid from frontier cells (unexplored reachable cells)
 * - Follows the gradient to efficiently explore all cells
 * - Naturally handles backtracking through explored regions
 * - Avoids goal cells until exploration is complete
 */
class FloodFillSolver {
 public:
  /**
   * @brief Explores the maze using Flood-Fill algorithm.
   *
   * Uses distance gradient to visit all reachable cells efficiently.
   * Goal cells are avoided until exploration is complete.
   *
   * @param mouse Reference to the InternalMouse instance representing the
   * mouse's state.
   * @param api Reference to the IAPIInterface instance for interacting with the maze
   * display (platform-agnostic).
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   */
  static void explore(InternalMouse& mouse, IAPIInterface& api, bool diagonalsAllowed);

 private:
  // Distance grid for flood-fill
  static int distanceGrid_[MAZE_SIZE][MAZE_SIZE];
  static const int INFINITY_DIST = 9999;

  /**
   * @brief Initializes the distance grid with infinity.
   *
   * @param mouse Reference to the InternalMouse instance.
   */
  static void initializeDistanceGrid(InternalMouse& mouse);

  /**
   * @brief Updates distances using wavefront propagation from frontier cells.
   *
   * Frontier cells are unexplored cells reachable from explored cells.
   * Only propagates through accessible paths (no walls between cells).
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   */
  static void updateDistances(InternalMouse& mouse, bool diagonalsAllowed);

  /**
   * @brief Calculates turn cost for cardinal directions only.
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param neighbor The neighbor cell to evaluate.
   * @return 0 for straight, 1 for 90° turn, 2 for 180° turn.
   */
  static int getTurnCost(InternalMouse& mouse, MazeNode* neighbor);

  /**
   * @brief Gets the best neighbor to move to.
   *
   * Prioritizes: 1) Unexplored with least turning, 2) Explored with lowest
   * (distance + turn_cost) score. Falls back to explored cells for backtracking.
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param current Current cell the robot is on.
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   * @return MazeNode* Best neighbor to move to, or nullptr if none.
   */
  static MazeNode* getBestNeighbor(InternalMouse& mouse, MazeNode* current,
                                    bool diagonalsAllowed);

  /**
   * @brief Moves directly to an adjacent cell without pathfinding.
   *
   * Calculates the required turn and moves forward one cell.
   *
   * @param api Reference to the IAPIInterface for movement commands.
   * @param mouse Reference to the InternalMouse instance.
   * @param target Pointer to the adjacent target cell.
   */
  static void moveDirectlyToAdjacent(IAPIInterface& api, InternalMouse& mouse,
                                     MazeNode* target);

  /**
   * @brief Cascades dead-end marking from a cell.
   *
   * When a cell has 3 walls, it's marked as explored (since the 4th
   * direction is the entry path, making it a dead-end). This may cause
   * adjacent cells to become dead-ends, triggering a cascade.
   *
   * @param mouse Reference to the InternalMouse instance.
   * @param api Reference to the IAPIInterface for visualization.
   * @param cell The cell to start cascade from.
   * @param diagonalsAllowed Whether diagonal movements are permitted.
   */
  static void markDeadEndCascade(InternalMouse& mouse, IAPIInterface& api,
                                  MazeNode* cell, bool diagonalsAllowed);

  /**
   * @brief Updates the distance text display for all cells.
   *
   * Shows the flood-fill distance values on each cell for visualization.
   *
   * @param api Reference to the IAPIInterface for text display.
   */
  static void updateDistanceDisplay(IAPIInterface& api);
};

#endif  // FLOODFILLSOLVER_H
