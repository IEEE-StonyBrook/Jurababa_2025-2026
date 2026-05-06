#ifndef NAVIGATION_FLOOD_FILL_H
#define NAVIGATION_FLOOD_FILL_H

#include <queue>
#include <vector>

#include "common/log.h"
#include "config/config.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"

class Mouse;

/**
 * @brief Flood-Fill maze exploration algorithm
 *
 * Uses distance gradient from unexplored frontier cells to efficiently
 * explore all reachable cells. Naturally handles backtracking through
 * explored regions.
 */
class FloodFill
{
  public:
    /**
     * @brief Explores the maze using Flood-Fill algorithm
     * @param mouse Reference to the MazeMouse instance
     * @param api Reference to the Mouse for movement commands
     * @param diagonals Whether diagonal movements are permitted
     */
    static void explore(MazeMouse& mouse, Mouse& api, bool diagonals);

  private:
    static int       distance_grid_[MAZE_SIZE][MAZE_SIZE];
    static const int INF_DIST = 9999;

    static void  initDistanceGrid(MazeMouse& mouse);
    static void  updateDistances(MazeMouse& mouse, bool diagonals);
    static int   turnCost(MazeMouse& mouse, Cell* neighbor);
    static Cell* bestNeighbor(MazeMouse& mouse, Cell* current, bool diagonals);
    static void  moveToAdjacent(Mouse& api, MazeMouse& mouse, Cell* target);
    static void  markDeadEnds(MazeMouse& mouse, Mouse& api, Cell* cell, bool diagonals);
    static void  updateDisplay(Mouse& api);
};

#endif
