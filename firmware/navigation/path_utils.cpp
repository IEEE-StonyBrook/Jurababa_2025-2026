#include "navigation/path_utils.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "common/tof_wall_utils.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/a_star.h"
#include "navigation/diagonalizer.h"
#include "navigation/path_converter.h"

#ifndef SIMULATOR_BUILD
#include "control/robot.h"
#endif

namespace PathUtils
{
namespace
{
struct ExploredNode
{
    Cell* cell;
    float g_cost;
    float f_cost;

    bool operator<(const ExploredNode& other) const { return f_cost > other.f_cost; }
};

float manhattan(Cell* from, Cell* to)
{
    return static_cast<float>(std::abs(from->x() - to->x()) + std::abs(from->y() - to->y()));
}

std::string wallBits(Cell* cell)
{
    if (cell == nullptr)
        return "N=? E=? S=? W=?";

    return "N=" + std::to_string(cell->hasWall('N')) + " E=" + std::to_string(cell->hasWall('E')) +
           " S=" + std::to_string(cell->hasWall('S')) + " W=" + std::to_string(cell->hasWall('W'));
}

bool readWallSample(API* api, int16_t& left_mm, int16_t& front_mm, int16_t& right_mm);

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
    int16_t left_mm  = 0;
    int16_t front_mm = 0;
    int16_t right_mm = 0;
    readWallSample(api, left_mm, front_mm, right_mm);
    const tof_wall::WallState wall_state = tof_wall::evaluate(left_mm, front_mm, right_mm);
    const float               steering_preview =
        wall_state.steering_allowed
            ? tof_wall::steeringAdjustmentDegps(wall_state.side_error_norm, 0.0f)
            : 0.0f;

    LOG_ERROR("No-path ToF L=" + std::to_string(left_mm) + " F=" + std::to_string(front_mm) +
              " R=" + std::to_string(right_mm) + " inferred_walls L=" +
              std::to_string(wall_state.left_wall) + " F=" + std::to_string(wall_state.front_wall) +
              " R=" + std::to_string(wall_state.right_wall) +
              " src=" + tof_wall::sourceName(wall_state.source) +
              " left_err_norm=" + std::to_string(wall_state.left_error_norm) +
              " right_err_norm=" + std::to_string(wall_state.right_error_norm) +
              " side_error_norm=" + std::to_string(wall_state.side_error_norm) +
              " steering_degps=" + std::to_string(steering_preview) +
              " front_blocked=" + std::to_string(wall_state.front_blocked));
#endif

    if (api != nullptr)
    {
        LOG_ERROR("No-path API wallLeft=" + std::to_string(api->wallLeft()) +
                  " wallFront=" + std::to_string(api->wallFront()) +
                  " wallRight=" + std::to_string(api->wallRight()));
    }
}

bool readWallSample(API* api, int16_t& left_mm, int16_t& front_mm, int16_t& right_mm)
{
    // api->wallSample falls through to live Robot getters on hardware
    // (FirmwareApi::wallSample uses robot()->{left,front,right}Distance()).
    if (api != nullptr && api->wallSample(left_mm, front_mm, right_mm))
        return true;
    return false;
}

void logSearchTrace(API* api, Mouse* mouse)
{
    if (mouse == nullptr)
        return;

    Cell* cell = mouse->currentCell();
    if (cell == nullptr)
        return;

#ifndef SIMULATOR_BUILD
    int16_t left_mm  = 0;
    int16_t front_mm = 0;
    int16_t right_mm = 0;
    readWallSample(api, left_mm, front_mm, right_mm);
    const tof_wall::WallState wall_state = tof_wall::evaluate(left_mm, front_mm, right_mm);
    const float               steering_preview =
        wall_state.steering_allowed
            ? tof_wall::steeringAdjustmentDegps(wall_state.side_error_norm, 0.0f)
            : 0.0f;
    Robot*      robot = api != nullptr ? api->robot() : nullptr;
    const float yaw   = robot != nullptr ? robot->angle() : 0.0f;

    LOG_INFO("SEARCH cell=(" + std::to_string(cell->x()) + "," + std::to_string(cell->y()) +
             ") heading=" + mouse->currentDirection() + " ToF L/F/R=" + std::to_string(left_mm) +
             "/" + std::to_string(front_mm) + "/" + std::to_string(right_mm) +
             " walls L/F/R=" + std::to_string(wall_state.left_wall) + "/" +
             std::to_string(wall_state.front_wall) + "/" + std::to_string(wall_state.right_wall) +
             " src=" + tof_wall::sourceName(wall_state.source) +
             " Lerr/Rerr=" + std::to_string(wall_state.left_error_norm) + "/" +
             std::to_string(wall_state.right_error_norm) +
             " side_error_norm=" + std::to_string(wall_state.side_error_norm) +
             " steering_degps=" + std::to_string(steering_preview) +
             " allowed=" + std::to_string(wall_state.steering_allowed) + " front_blocked=" +
             std::to_string(wall_state.front_blocked) + " yaw=" + std::to_string(yaw));
#else
    LOG_INFO("SEARCH cell=(" + std::to_string(cell->x()) + "," + std::to_string(cell->y()) +
             ") heading=" + mouse->currentDirection());
#endif
}

void detectWallsFromSample(API* api, Mouse* mouse)
{
    if (api == nullptr || mouse == nullptr)
        return;

    detectWalls(*api, *mouse);
    logSearchTrace(api, mouse);
    api->clearWallSample();
}

std::vector<Cell*> reconstructExploredPath(Mouse*                                 mouse,
                                           const std::vector<std::vector<Cell*>>& parents,
                                           Cell* start, Cell* end)
{
    std::vector<Cell*> path;
    Cell*              node = end;

    while (node != nullptr && node != start)
    {
        path.push_back(node);
        node = parents[node->x()][node->y()];
    }

    if (node != start)
    {
        LOG_ERROR("Explored speed path reconstruction failed.");
        return {};
    }

    std::reverse(path.begin(), path.end());

    for (Cell* cell : path)
    {
        if (!cell->explored())
        {
            LOG_ERROR("Explored speed path rejected unexplored cell (" + std::to_string(cell->x()) +
                      "," + std::to_string(cell->y()) + ")");
            return {};
        }
    }

    return path;
}

std::vector<Cell*> exploredPathTo(Mouse* mouse, Cell* end)
{
    Cell* start = mouse->currentCell();
    if (start == nullptr || end == nullptr)
        return {};

    if (!start->explored() || !end->explored())
    {
        LOG_ERROR("Explored speed path requires explored start and goal cells.");
        return {};
    }

    if (start == end)
        return {};

    const int width  = mouse->mazeWidth();
    const int height = mouse->mazeHeight();

    std::vector<std::vector<float>> g_costs(
        width, std::vector<float>(height, std::numeric_limits<float>::infinity()));
    std::vector<std::vector<Cell*>>   parents(width, std::vector<Cell*>(height, nullptr));
    std::vector<std::vector<bool>>    closed(width, std::vector<bool>(height, false));
    std::priority_queue<ExploredNode> open;

    g_costs[start->x()][start->y()] = 0.0f;
    open.push({start, 0.0f, manhattan(start, end)});

    while (!open.empty())
    {
        ExploredNode current = open.top();
        open.pop();

        if (current.cell == end)
            return reconstructExploredPath(mouse, parents, start, end);

        if (closed[current.cell->x()][current.cell->y()])
            continue;
        closed[current.cell->x()][current.cell->y()] = true;

        for (Cell* neighbor : mouse->cellNeighbors(current.cell, /*include_diagonal=*/false))
        {
            if (!neighbor->explored())
                continue;
            if (closed[neighbor->x()][neighbor->y()])
                continue;
            if (!mouse->canMoveBetween(current.cell, neighbor, /*diagonals=*/false))
                continue;

            const float new_g = current.g_cost + 1.0f;
            if (new_g < g_costs[neighbor->x()][neighbor->y()])
            {
                g_costs[neighbor->x()][neighbor->y()] = new_g;
                parents[neighbor->x()][neighbor->y()] = current.cell;
                open.push({neighbor, new_g, new_g + manhattan(neighbor, end)});
            }
        }
    }

    LOG_ERROR("No explored-only speed path found.");
    return {};
}

std::vector<Cell*> bestExploredPath(Mouse* mouse, const std::vector<std::array<int, 2>>& goals)
{
    std::vector<Cell*> best_path;
    float              best_cost = std::numeric_limits<float>::infinity();

    for (const auto& goal : goals)
    {
        Cell* goal_cell = mouse->cellAt(goal[0], goal[1]);
        if (goal_cell == nullptr || !goal_cell->explored())
            continue;

        std::vector<Cell*> path = exploredPathTo(mouse, goal_cell);
        if (path.empty())
            continue;

        float cost = static_cast<float>(path.size());
        if (cost < best_cost)
        {
            best_path = path;
            best_cost = cost;
        }
    }

    return best_path;
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
                  bool diagonals, bool all_explored, bool avoid_goals, bool start_at_wall_check)
{
    Cell* current = mouse->currentCell();
    AStar a_star(mouse);
    bool  at_start_wall_check = start_at_wall_check;

    if (all_explored)
    {
        LOG_INFO("Marking all explored for debug run.");
        setAllExplored(mouse);
    }

    if (at_start_wall_check)
        api->captureWallSample();

    while (true)
    {
        detectWallsFromSample(api, mouse);
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
        {
            if (at_start_wall_check)
            {
                if (!api->center_from_wall_check())
                {
                    LOG_ERROR("SEARCH failed moving from wall-check pose to cell center.");
                    return false;
                }
                at_start_wall_check = false;
            }
            api->finish_search_move();
            break;
        }

        // Get path from A*
        std::vector<Cell*> cell_path = a_star.cellPath(
            const_cast<std::vector<std::array<int, 2>>&>(goals), diagonals, !avoid_goals);

        if (cell_path.empty())
        {
            LOG_ERROR("No path found!");
            logNoPathDiagnostics(api, mouse);
            if (at_start_wall_check)
            {
                if (!api->center_from_wall_check())
                {
                    LOG_ERROR("SEARCH failed moving from wall-check pose to cell center.");
                    return false;
                }
                at_start_wall_check = false;
            }
            api->finish_search_move();
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

            if (move == "F")
            {
                bool motion_started = false;
                if (at_start_wall_check)
                {
                    motion_started      = api->search_start_from_wall_check();
                    at_start_wall_check = false;
                }
                else
                {
                    motion_started = api->search_advance();
                }

                if (!motion_started)
                {
                    LOG_ERROR("SEARCH motion command failed before wall-check sample.");
                    api->clear_search_move();
                    return false;
                }
                current = mouse->currentCell();
                break;
            }

            if (at_start_wall_check)
            {
                if (!api->center_from_wall_check())
                {
                    LOG_ERROR("SEARCH failed moving from wall-check pose to cell center.");
                    return false;
                }
                at_start_wall_check = false;
            }

            api->executeSequence(move);
            api->clear_search_move();
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

bool traversePath(API* api, Mouse* mouse, const std::vector<std::array<int, 2>>& goals,
                  bool diagonals, bool all_explored, bool avoid_goals)
{
    return traversePath(api, mouse, goals, diagonals, all_explored, avoid_goals,
                        /*start_at_wall_check=*/false);
}

bool traverseExploredPath(API* api, Mouse* mouse, const std::vector<std::array<int, 2>>& goals)
{
    if (api == nullptr || mouse == nullptr)
        return false;

    std::vector<Cell*> best_path = bestExploredPath(mouse, goals);
    if (best_path.empty())
    {
        LOG_ERROR("No explored-only cardinal speed run path available.");
        return false;
    }

    colorPath(api, best_path);

    std::string lfr =
        PathConverter::buildLFR(mouse->currentCell(), mouse->currentDirectionArray(), best_path);
    LOG_INFO("Explored Cardinal A* LFR Path: " + lfr);
    api->executeSequence(lfr);
    return true;
}

bool traverseExploredDiagonalPath(API* api, Mouse* mouse,
                                  const std::vector<std::array<int, 2>>& goals)
{
    if (api == nullptr || mouse == nullptr)
        return false;

    std::vector<Cell*> best_path = bestExploredPath(mouse, goals);
    if (best_path.empty())
    {
        LOG_ERROR("No explored-only diagonal speed run path available.");
        return false;
    }

    colorPath(api, best_path);

    std::string lfr =
        PathConverter::buildLFR(mouse->currentCell(), mouse->currentDirectionArray(), best_path);
    LOG_INFO("Explored A* LFR Path: " + lfr);

    std::string diag = Diagonalizer::diagonalize(lfr);
    LOG_INFO("Explored Diagonalized Path: " + diag);
    api->executeSequence(diag);
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
