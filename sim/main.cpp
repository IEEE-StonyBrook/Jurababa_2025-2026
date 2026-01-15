/**
 * @file main.cpp
 * @brief Micromouse Three-Phase Competition Simulator
 *
 * Simulates a competition run with mms (Micromouse Simulator):
 *   1. Exploration: Flood-fill search to map maze
 *   2. Return: A* without diagonals back to start
 *   3. Speed Run: A* with diagonals for optimal time
 */

#include <array>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "config/geometry.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/flood_fill.h"
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

    // Phase 1: EXPLORATION - Map entire maze using flood-fill search
    LOG_INFO("Phase 1: Exploration (Flood-Fill)");
    api.setPhaseColor('y');
    FloodFill::explore(mouse, api, false);
    LOG_INFO("Maze fully explored\n");

    // Phase 2: RETURN - Go back to start using discovered layout (no diagonals)
    LOG_INFO("Phase 2: Return to Start (A* without diagonals)");
    api.clearAllColor();
    api.setPhaseColor('c');
    PathUtils::setAllExplored(&mouse);
    PathUtils::traversePath(&api, &mouse, {start}, false, true, false);
    LOG_INFO("Returned to start\n");

    // Phase 3: SPEED RUN - Optimal path to goal with diagonals
    LOG_INFO("Phase 3: Speed Run (A* with diagonals)");
    api.setPhaseColor('G');
    PathUtils::traversePath(&api, &mouse, goals, true, true, false);
    LOG_INFO("Speed run complete\n");

    LOG_INFO("=== SIMULATION COMPLETE ===");
    return 0;
}
