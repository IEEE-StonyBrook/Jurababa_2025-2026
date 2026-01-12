#ifndef NAVIGATION_A_STAR_H
#define NAVIGATION_A_STAR_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "common/log.h"
#include "maze/maze.h"
#include "maze/mouse.h"

class API;

/**
 * @brief A* pathfinding algorithm for maze navigation
 *
 * Finds optimal paths using A* with configurable heuristics.
 * Supports both cardinal and diagonal movement modes.
 */
class AStar
{
  public:
    explicit AStar(Mouse* mouse);

    /**
     * @brief Get LFR path string to goal cells
     * @param goals Target cell coordinates
     * @param diagonals Allow diagonal movement
     * @param pass_goals Allow path through goal cells
     * @return LFR-formatted path string (e.g., "F#L#F#F#R#F")
     */
    std::string pathTo(std::vector<std::array<int, 2>> goals,
                       bool diagonals = false,
                       bool pass_goals = false);

    /**
     * @brief Get cell-based path to goal cells
     * @return Vector of Cell pointers representing the path
     */
    std::vector<Cell*> cellPath(std::vector<std::array<int, 2>> goals,
                                bool diagonals = false,
                                bool pass_goals = false);

    float lastPathCost() const { return total_cost_; }

  private:
    std::vector<Cell*> findBestPath(std::vector<std::array<int, 2>> goals,
                                    bool diagonals, bool pass_goals);
    std::vector<Cell*> findPathTo(Cell* end, bool diagonals, bool pass_goals);
    std::vector<Cell*> reconstructPath(Cell* start, Cell* end);

    static std::string pathToString(const std::vector<Cell*>& path);
    static float heuristic(Cell* from, Cell* to);

    Mouse* mouse_;
    float total_cost_ = 0.0f;
};

#endif
