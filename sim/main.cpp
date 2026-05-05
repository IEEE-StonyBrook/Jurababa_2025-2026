/**
 * @file main.cpp
 * @brief Micromouse Three-Phase Competition Simulator
 *
 * Simulates a competition run with mms (Micromouse Simulator):
 *   1. Search: iterative A* without diagonals to the goal
 *   2. Return: iterative A* without diagonals back to start
 *   3. Speed Run: diagonalized A* for optimal time
 */

#include <array>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "config/geometry.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/path_utils.h"

int main()
{
    LOG_INFO("=== MICROMOUSE COMPETITION SIMULATOR ===\n");

    // Initialize 16x16 maze with center goal
    std::array<int, 2>              start = {0, 0};
    std::vector<std::array<int, 2>> goals = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

    Maze  maze(MAZE_SIZE, MAZE_SIZE);
    Mouse mouse(start, "n", goals, &maze);
    API   api(&mouse);
    api.run_on_simulator = true;
    api.setUp(start, goals);

    // Phase 1: SEARCH - Iterative A* to the goal, sensing walls and replanning
    // whenever the path reaches an unexplored cell.
    LOG_INFO("Phase 1: Search to Goal (Iterative A* without diagonals)");
    api.setPhaseColor('y');
    PathUtils::traversePath(&api, &mouse, goals, /*diagonals=*/false,
                            /*all_explored=*/false, /*avoid_goals=*/false);
    LOG_INFO("Reached goal\n");

    // Phase 2: RETURN - Go back to start using iterative A* with no diagonals.
    LOG_INFO("Phase 2: Return to Start (Iterative A* without diagonals)");
    api.clearAllColor();
    api.setPhaseColor('c');
    PathUtils::traversePath(&api, &mouse, {start}, /*diagonals=*/false,
                            /*all_explored=*/false, /*avoid_goals=*/false);
    LOG_INFO("Returned to start\n");

    // Phase 3: SPEED RUN - Diagonalized A* over the discovered wall map.
    LOG_INFO("Phase 3: Speed Run (Diagonalized A*)");
    api.setPhaseColor('G');
    PathUtils::traverseExploredDiagonalPath(&api, &mouse, goals);
    LOG_INFO("Speed run complete\n");

    LOG_INFO("=== SIMULATION COMPLETE ===");
    return 0;
}
