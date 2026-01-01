#include <stdio.h>

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "Common/LogSystem.h"
#include "Navigation/AStarSolver.h"
#include "Navigation/FrontierBasedSearchSolver.h"
#include "Platform/Simulator/API.h"
#include "Navigation/PathUtils.h"

int main()
{
    const bool RUN_ON_SIMULATOR = true;

    // Universal objects
    std::array<int, 2>              startCell = {0, 0};
    std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

    // Mouse logic objects
    MazeGraph     maze(16, 16);
    InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);
    API_SIMULATOR api(&mouse, RUN_ON_SIMULATOR);
    api.setUp(startCell, goalCells);
    api.printMaze();

    traversePathIteratively(&api, &mouse, goalCells, true, false, false);
    traversePathIteratively(&api, &mouse, {startCell}, true, false, false);

    return 0;
}