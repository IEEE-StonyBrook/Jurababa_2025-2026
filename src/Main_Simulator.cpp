/**
 * Main_Simulator.cpp - Micromouse Three-Phase Competition Simulator
 *
 * Simulates a competition run:
 *   1. Exploration: Frontier-based search to map maze
 *   2. Return: A* without diagonals back to start
 *   3. Speed Run: A* with diagonals for optimal time
 */

#include <array>
#include <vector>

#include "Common/LogSystem.h"
#include "Maze/InternalMouse.h"
#include "Maze/MazeGraph.h"
#include "Navigation/FrontierBasedSearchSolver.h"
#include "Navigation/PathUtils.h"
#include "Platform/Simulator/API.h"

int main()
{
    LOG_INFO("=== MICROMOUSE COMPETITION SIMULATOR ===\n");

    // Initialize 16x16 maze with center goal
    std::array<int, 2>              start = {0, 0};
    std::vector<std::array<int, 2>> goal  = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

    MazeGraph     maze(16, 16);
    InternalMouse mouse(start, "n", goal, &maze);
    API_SIMULATOR api(&mouse, true);
    api.setUp(start, goal);

    // Phase 1: EXPLORATION - Map entire maze using frontier-based search
    LOG_INFO("Phase 1: Exploration (Frontier-Based)");
    api.setPhaseColor('y');  // Yellow for exploration
    FrontierBased::explore(mouse, api, false);
    LOG_INFO("✓ Maze fully explored\n");

    // Phase 2: RETURN - Go back to start using discovered layout (no diagonals)
    LOG_INFO("Phase 2: Return to Start (A* without diagonals)");
    api.setPhaseColor('c');  // Cyan for return path
    setAllExplored(&mouse);
    traversePathIteratively(&api, &mouse, {start}, false, true, false);
    LOG_INFO("✓ Returned to start\n");

    // Phase 3: SPEED RUN - Optimal path to goal with diagonals
    LOG_INFO("Phase 3: Speed Run (A* with diagonals)");
    api.setPhaseColor('G');  // Green for speed run
    traversePathIteratively(&api, &mouse, goal, true, true, false);
    LOG_INFO("✓ Speed run complete\n");

    LOG_INFO("=== SIMULATION COMPLETE ===");
    return 0;
}
