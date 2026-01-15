#include "navigation/path_utils.h"

#include <sstream>
#include <string>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/a_star.h"
#include "navigation/diagonalizer.h"
#include "navigation/path_converter.h"

namespace PathUtils
{

void executePath(API* api, const std::string& lfr_path)
{
    std::stringstream ss(lfr_path);
    std::string       token;

    std::vector<std::string> tokens;
    while (std::getline(ss, token, '#'))
    {
        if (!token.empty())
            tokens.push_back(token);
    }

    for (const std::string& t : tokens)
    {
        if (t == "R")
            api->turnRight90();
        else if (t == "L")
            api->turnLeft90();
        else if (t == "F")
            api->moveForward();
        else if (t == "R45")
            api->turnRight45();
        else if (t == "L45")
            api->turnLeft45();
        else if (t == "FH")
            api->moveForwardHalf();
        else if (t == "GMF")
            api->ghostMoveForward(1);
        else
            LOG_ERROR("PathUtils: Unknown token: " + t);
    }
}

void setAllExplored(Mouse* mouse)
{
    int cols = mouse->mazeWidth();
    int rows = mouse->mazeHeight();

    for (int x = 0; x < cols; x++)
    {
        for (int y = 0; y < rows; y++)
        {
            mouse->cellAt(x, y)->markExplored();
        }
    }
}

bool traversePath(API* api, Mouse* mouse, const std::vector<std::array<int, 2>>& goals,
                  bool diagonals, bool all_explored, bool avoid_goals)
{
    Cell* current = mouse->currentCell();
    AStar a_star(mouse);

    if (all_explored)
    {
        LOG_INFO("Marking all explored for debug run.");
        setAllExplored(mouse);
    }

    while (true)
    {
        current->markExplored();

        // Check if we've reached a goal
        bool reached_goal = false;
        for (const auto& goal : goals)
        {
            if (current->x() == goal[0] && current->y() == goal[1])
            {
                reached_goal = true;
                break;
            }
        }
        if (reached_goal)
            break;

        // Detect walls at current position
        detectWalls(*api, *mouse);

        // Get path from A*
        std::vector<Cell*> cell_path = a_star.cellPath(
            const_cast<std::vector<std::array<int, 2>>&>(goals), diagonals, !avoid_goals);

        if (cell_path.empty())
        {
            LOG_ERROR("No path found!");
            return false;
        }

        colorPath(api, cell_path);

        // Convert to LFR format
        std::string lfr = PathConverter::buildLFR(mouse->currentCell(),
                                                  mouse->currentDirectionArray(), cell_path);

        LOG_INFO("A* LFR Path: " + lfr);

        // Diagonalize and execute full path if maze is explored
        if (all_explored && diagonals)
        {
            std::string diag = Diagonalizer::diagonalize(lfr);
            LOG_INFO("Diagonalized Path: " + diag);
            executePath(api, diag);
            return true;
        }

        // Execute step-by-step, re-planning on unexplored cells
        std::stringstream ss(lfr);
        std::string       move;
        while (std::getline(ss, move, '#'))
        {
            if (move.empty())
                continue;

            api->executeSequence(move);
            current = mouse->currentCell();

            if (!current->explored())
            {
                LOG_DEBUG("[RE-CALC] Hit unexplored cell at (" + std::to_string(current->x()) +
                          "," + std::to_string(current->y()) + ")");
                break;
            }

            LOG_DEBUG("[RE-USE] Continuing with path token: " + move);
        }

        LOG_DEBUG("Breaking to re-calc path");
    }

    return true;
}

void detectWalls(API& api, Mouse& mouse)
{
    Cell* cell = mouse.currentCell();
    int   x    = cell->x();
    int   y    = cell->y();

    if (api.wallFront())
    {
        api.setWall(x, y, mouse.directionAsString(mouse.currentDirectionArray()));
    }
    if (api.wallLeft())
    {
        api.setWall(x, y, mouse.directionLeft());
    }
    if (api.wallRight())
    {
        api.setWall(x, y, mouse.directionRight());
    }
}

void colorPath(API* api, const std::vector<Cell*>& path)
{
    char color = api->phaseColor();
    for (Cell* cell : path)
    {
        api->setColor(cell->x(), cell->y(), color);
    }
}

} // namespace PathUtils
