#include "navigation/path_utils.h"

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "config/sensors.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/a_star.h"
#include "navigation/diagonalizer.h"
#include "navigation/path_converter.h"

#ifndef SIMULATOR_BUILD
#include "app/multicore.h"
#endif

namespace PathUtils
{
namespace
{
std::string wallBits(Cell* cell)
{
    if (cell == nullptr)
        return "N=? E=? S=? W=?";

    return "N=" + std::to_string(cell->hasWall('N')) + " E=" + std::to_string(cell->hasWall('E')) +
           " S=" + std::to_string(cell->hasWall('S')) + " W=" + std::to_string(cell->hasWall('W'));
}

#ifndef SIMULATOR_BUILD
bool wallFromTofMm(int16_t mm, int threshold_mm)
{
    return mm > 0 && mm < threshold_mm;
}
#endif

void logNoPathDiagnostics(API* api, Mouse* mouse)
{
    if (mouse == nullptr)
        return;

    Cell* cell = mouse->currentCell();
    if (cell != nullptr)
    {
        LOG_ERROR("No-path current cell=(" + std::to_string(cell->x()) + "," +
                  std::to_string(cell->y()) + ") heading=" + mouse->currentDirection() +
                  " cell_walls " + wallBits(cell));
    }

#ifndef SIMULATOR_BUILD
    SensorData snap;
    SensorHub::snapshot(snap);
    bool left_wall  = wallFromTofMm(snap.tof_left_mm, TOF_LEFT_WALL_THRESHOLD_MM);
    bool front_wall = wallFromTofMm(snap.tof_front_mm, TOF_FRONT_WALL_THRESHOLD_MM);
    bool right_wall = wallFromTofMm(snap.tof_right_mm, TOF_RIGHT_WALL_THRESHOLD_MM);

    LOG_ERROR("No-path ToF L=" + std::to_string(snap.tof_left_mm) + " F=" +
              std::to_string(snap.tof_front_mm) + " R=" + std::to_string(snap.tof_right_mm) +
              " inferred_walls L=" + std::to_string(left_wall) +
              " F=" + std::to_string(front_wall) + " R=" + std::to_string(right_wall));
#endif

    if (api != nullptr)
    {
        LOG_ERROR("No-path API wallLeft=" + std::to_string(api->wallLeft()) +
                  " wallFront=" + std::to_string(api->wallFront()) +
                  " wallRight=" + std::to_string(api->wallRight()));
    }
}
} // namespace

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
            logNoPathDiagnostics(api, mouse);
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
            api->executeSequence(diag);
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
