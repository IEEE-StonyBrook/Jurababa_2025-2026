#ifndef NAVIGATION_FLOOD_FILL_H
#define NAVIGATION_FLOOD_FILL_H

#include <queue>
#include <vector>

#include "common/log.h"
#include "config/config.h"
#include "maze/maze.h"
#include "maze/mouse.h"

class API;

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
     * @param mouse Reference to the Mouse instance
     * @param api Reference to the API for movement commands
     * @param diagonals Whether diagonal movements are permitted
     */
    static void explore(Mouse& mouse, API& api, bool diagonals);

  private:
    static int distance_grid_[MAZE_SIZE][MAZE_SIZE];
    static const int INF_DIST = 9999;

    static void initDistanceGrid(Mouse& mouse);
    static void updateDistances(Mouse& mouse, bool diagonals);
    static int turnCost(Mouse& mouse, Cell* neighbor);
    static Cell* bestNeighbor(Mouse& mouse, Cell* current, bool diagonals);
    static void moveToAdjacent(API& api, Mouse& mouse, Cell* target);
    static void markDeadEnds(Mouse& mouse, API& api, Cell* cell, bool diagonals);
    static void updateDisplay(API& api);
};

#endif
