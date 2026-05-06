#include "navigation/path_utils.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <queue>
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
namespace
{
struct ExploredNode
{
    Cell* cell;
    float g_cost;
    float f_cost;

    bool operator<(const ExploredNode& other) const { return f_cost > other.f_cost; }
};

class MotionSequenceGuard
{
  public:
    explicit MotionSequenceGuard(API* api) : api_(api)
    {
        if (api_ != nullptr)
            api_->begin_motion_sequence();
    }

    ~MotionSequenceGuard()
    {
        if (api_ != nullptr)
            api_->end_motion_sequence();
    }

    MotionSequenceGuard(const MotionSequenceGuard&)            = delete;
    MotionSequenceGuard& operator=(const MotionSequenceGuard&) = delete;

  private:
    API* api_;
};

float manhattan(Cell* from, Cell* to)
{
    return static_cast<float>(std::abs(from->x() - to->x()) + std::abs(from->y() - to->y()));
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
    (void)diagonals;
    (void)all_explored;
    (void)avoid_goals;
    (void)start_at_wall_check;
    if (api == nullptr || mouse == nullptr)
        return false;
    return api->search_to(goals);
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

    MotionSequenceGuard motion_sequence(api);

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

    MotionSequenceGuard motion_sequence(api);

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
