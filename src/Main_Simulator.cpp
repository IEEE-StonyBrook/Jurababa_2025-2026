#include <stdio.h>

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "../Include/Common/LogSystem.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Navigation/FrontierBasedSearchSolver.h"
#include "../Include/Platform/Simulator/API.h"
#include "PathUtils.h"

int main()
{
    const bool RUN_ON_SIMULATOR = true;

    // Universal objects
    LogSystem                       logSystem;
    std::array<int, 2>              startCell = {0, 0};
    std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

    // Mouse logic objects
    MazeGraph     maze(16, 16);
    InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);
    API_SIMULATOR           api(&mouse, RUN_ON_SIMULATOR);
    api.setUp(startCell, goalCells);
    api.printMaze();

    FrontierBased frontierSolver;
    frontierSolver.explore(mouse, api, true);

    AStarSolver aStar(&mouse);
    std::string lfrPath = aStar.go({startCell}, true, true);
    interpretLFRPath(&api, lfrPath);

    std::string lfrPath2 = aStar.go(goalCells, true, true);
    interpretLFRPath(&api, lfrPath2);

    return 0;
}