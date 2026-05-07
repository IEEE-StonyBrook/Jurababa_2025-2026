#include "navigation/path_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "app/mouse.h"
#include "common/log.h"
#include "common/utils.h"
#include "config/geometry.h"
#include "control/motion.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/a_star.h"
#include "navigation/diagonalizer.h"
#include "navigation/path_converter.h"

namespace PathUtils
{
namespace
{
constexpr float kSpeedRunYawPreflightToleranceDeg = 20.0f;

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
    explicit MotionSequenceGuard(Mouse* mouse) : mouse_(mouse)
    {
        if (mouse_ != nullptr)
            mouse_->begin_motion_sequence();
    }

    ~MotionSequenceGuard()
    {
        if (mouse_ != nullptr)
            mouse_->end_motion_sequence();
    }

    MotionSequenceGuard(const MotionSequenceGuard&)            = delete;
    MotionSequenceGuard& operator=(const MotionSequenceGuard&) = delete;

  private:
    Mouse* mouse_;
};

class MouseHeadingHoldGuard
{
  public:
    explicit MouseHeadingHoldGuard(Mouse* mouse) : mouse_(mouse)
    {
        if (mouse_ != nullptr)
            mouse_->begin_maze_heading_hold();
    }

    ~MouseHeadingHoldGuard()
    {
        if (mouse_ != nullptr)
            mouse_->end_maze_heading_hold();
    }

    MouseHeadingHoldGuard(const MouseHeadingHoldGuard&)            = delete;
    MouseHeadingHoldGuard& operator=(const MouseHeadingHoldGuard&) = delete;

  private:
    Mouse* mouse_;
};

float manhattan(Cell* from, Cell* to)
{
    return static_cast<float>(std::abs(from->x() - to->x()) + std::abs(from->y() - to->y()));
}

std::string fixed1(float value)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
    return buffer;
}

std::string signedFixed1(float value)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%+.1f", static_cast<double>(value));
    return buffer;
}

bool parseForwardToken(const std::string& token, int& cells)
{
    if (token.empty() || token[0] != 'F')
        return false;
    if (token == "F")
    {
        cells = 1;
        return true;
    }

    if (token.size() < 2)
        return false;

    int parsed = 0;
    for (size_t i = 1; i < token.size(); ++i)
    {
        if (token[i] < '0' || token[i] > '9')
            return false;
        parsed = parsed * 10 + (token[i] - '0');
    }

    if (parsed <= 0)
        return false;
    cells = parsed;
    return true;
}

std::vector<std::string> splitSequence(const std::string& sequence)
{
    std::vector<std::string> tokens;
    std::string              token;
    for (char c : sequence)
    {
        if (c == '#')
        {
            if (!token.empty())
            {
                tokens.push_back(token);
                token.clear();
            }
        }
        else
        {
            token.push_back(c);
        }
    }
    if (!token.empty())
        tokens.push_back(token);
    return tokens;
}

std::string joinSequence(const std::vector<std::string>& tokens)
{
    std::string out;
    for (size_t i = 0; i < tokens.size(); ++i)
    {
        if (i > 0)
            out += "#";
        out += tokens[i];
    }
    return out;
}

std::string compressCardinalForwards(const std::string& sequence)
{
    std::vector<std::string> input = splitSequence(sequence);
    std::vector<std::string> output;

    int  pending_forwards = 0;
    auto flush_forwards   = [&]()
    {
        if (pending_forwards <= 0)
            return;
        output.push_back(pending_forwards == 1 ? std::string("F")
                                               : "F" + std::to_string(pending_forwards));
        pending_forwards = 0;
    };

    for (const std::string& token : input)
    {
        if (token == "F")
        {
            ++pending_forwards;
            continue;
        }
        flush_forwards();
        output.push_back(token);
    }
    flush_forwards();
    return joinSequence(output);
}

bool executeStartCenter(Mouse* mouse)
{
    LOG_INFO("FAST start_center distance_mm=" + fixed1(START_CENTER_DISTANCE_MM));
    return mouse->move_mm(START_CENTER_DISTANCE_MM);
}

float expectedYawForHeading(const std::string& heading)
{
    if (heading == "ne" || heading == "NE")
        return -45.0f;
    if (heading == "e" || heading == "E")
        return -90.0f;
    if (heading == "se" || heading == "SE")
        return -135.0f;
    if (heading == "s" || heading == "S")
        return 180.0f;
    if (heading == "sw" || heading == "SW")
        return 135.0f;
    if (heading == "w" || heading == "W")
        return 90.0f;
    if (heading == "nw" || heading == "NW")
        return 45.0f;
    return 0.0f;
}

bool speedRunYawPreflight(Mouse* mouse, MazeMouse* maze_mouse)
{
    if (mouse == nullptr || maze_mouse == nullptr || mouse->run_on_simulator)
        return true;

    Motion* motion = mouse->motion();
    if (motion == nullptr)
        return true;

    const std::string heading      = maze_mouse->currentDirection();
    const float       expected_yaw = expectedYawForHeading(heading);
    const float       actual_yaw   = motion->yaw_deg();
    const float       yaw_error    = utils::wrapAngle180(expected_yaw - actual_yaw);

    LOG_INFO("Speed run yaw preflight: heading=" + heading +
             " expected_yaw=" + fixed1(expected_yaw) + " actual_yaw=" + fixed1(actual_yaw) +
             " err=" + signedFixed1(yaw_error));

    if (std::fabs(yaw_error) <= kSpeedRunYawPreflightToleranceDeg)
        return true;

    LOG_ERROR("Speed run aborted: physical yaw does not match virtual heading.");
    return false;
}

bool atAnyGoal(MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals)
{
    Cell* current = maze_mouse != nullptr ? maze_mouse->currentCell() : nullptr;
    if (current == nullptr)
        return false;

    for (const auto& goal : goals)
    {
        if (current->x() == goal[0] && current->y() == goal[1])
            return true;
    }
    return false;
}

bool executeFastSequence(Mouse* mouse, const std::string& sequence, bool start_from_back_wall)
{
    if (mouse == nullptr)
        return false;

    MouseHeadingHoldGuard    heading_hold(mouse);
    std::vector<std::string> tokens        = splitSequence(sequence);
    bool                     start_pending = start_from_back_wall;

    for (const std::string& token : tokens)
    {
        if (mouse->haltRequested())
            return false;

        int forward_cells = 0;
        if (parseForwardToken(token, forward_cells))
        {
            const float start_extra_mm = start_pending ? START_CENTER_DISTANCE_MM : 0.0f;
            const float distance_mm =
                start_extra_mm + static_cast<float>(forward_cells) * CELL_SIZE_MM;
            LOG_INFO("FAST " + token +
                     " start=" + (start_pending ? std::string("true") : std::string("false")) +
                     " distance_mm=" + fixed1(distance_mm) +
                     " cells=" + std::to_string(forward_cells));
            if (!mouse->move_mm(distance_mm))
                return false;
            mouse->ghostMoveForward(forward_cells);
            start_pending = false;
            continue;
        }

        if (start_pending)
        {
            if (!executeStartCenter(mouse))
                return false;
            start_pending = false;
        }

        if (token == "FH")
        {
            mouse->moveForwardHalf();
        }
        else if (token == "L")
        {
            mouse->turnLeft90();
        }
        else if (token == "R")
        {
            mouse->turnRight90();
        }
        else if (token == "B")
        {
            mouse->turnRight90();
            if (mouse->haltRequested())
                return false;
            mouse->turnRight90();
        }
        else if (token == "L45")
        {
            mouse->turnLeft45();
        }
        else if (token == "R45")
        {
            mouse->turnRight45();
        }
        else if (token == "GMF" || token == "GFM")
        {
            if (token == "GFM")
                LOG_ERROR("FAST: Treating diagonalizer token GFM as GMF");
            mouse->ghostMoveForward(1);
        }
        else
        {
            LOG_ERROR("FAST: Unknown path token: " + token);
            return false;
        }

        if (mouse->haltRequested())
            return false;
    }

    return !mouse->haltRequested();
}

std::vector<Cell*> reconstructExploredPath(const std::vector<std::vector<Cell*>>& parents,
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

std::vector<Cell*> exploredPathTo(MazeMouse* maze_mouse, Cell* end)
{
    Cell* start = maze_mouse->currentCell();
    if (start == nullptr || end == nullptr)
        return {};

    if (!start->explored() || !end->explored())
    {
        LOG_ERROR("Explored speed path requires explored start and goal cells.");
        return {};
    }

    if (start == end)
        return {};

    const int width  = maze_mouse->mazeWidth();
    const int height = maze_mouse->mazeHeight();

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
            return reconstructExploredPath(parents, start, end);

        if (closed[current.cell->x()][current.cell->y()])
            continue;
        closed[current.cell->x()][current.cell->y()] = true;

        for (Cell* neighbor : maze_mouse->cellNeighbors(current.cell, /*include_diagonal=*/false))
        {
            if (!neighbor->explored())
                continue;
            if (closed[neighbor->x()][neighbor->y()])
                continue;
            if (!maze_mouse->canMoveBetween(current.cell, neighbor, /*diagonals=*/false))
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

std::vector<Cell*> bestExploredPath(MazeMouse*                             maze_mouse,
                                    const std::vector<std::array<int, 2>>& goals)
{
    std::vector<Cell*> best_path;
    float              best_cost = std::numeric_limits<float>::infinity();

    for (const auto& goal : goals)
    {
        Cell* goal_cell = maze_mouse->cellAt(goal[0], goal[1]);
        if (goal_cell == nullptr || !goal_cell->explored())
            continue;

        std::vector<Cell*> path = exploredPathTo(maze_mouse, goal_cell);
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

bool traversePath(Mouse* mouse, MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals,
                  bool diagonals, bool all_explored, bool avoid_goals)
{
    (void)diagonals;
    (void)all_explored;
    (void)avoid_goals;
    if (mouse == nullptr || maze_mouse == nullptr)
        return false;
    return mouse->search_to(goals);
}

bool traverseExploredPath(Mouse* mouse, MazeMouse* maze_mouse,
                          const std::vector<std::array<int, 2>>& goals, bool start_from_back_wall)
{
    if (mouse == nullptr || maze_mouse == nullptr)
        return false;

    if (atAnyGoal(maze_mouse, goals))
    {
        LOG_INFO("Already at goal; no speed-run path needed.");
        return true;
    }
    if (!speedRunYawPreflight(mouse, maze_mouse))
        return false;

    MotionSequenceGuard motion_sequence(mouse);
    const MazeMask      previous_mask = maze_mouse->mazeMask();
    maze_mouse->setMazeMask(MASK_CLOSED);

    std::vector<Cell*> best_path = bestExploredPath(maze_mouse, goals);
    if (best_path.empty())
    {
        maze_mouse->setMazeMask(previous_mask);
        LOG_ERROR("No explored-only cardinal speed run path available.");
        return false;
    }

    colorPath(mouse, best_path);

    std::string lfr = PathConverter::buildLFR(maze_mouse->currentCell(),
                                              maze_mouse->currentDirectionArray(), best_path);
    LOG_INFO("Explored Cardinal A* LFR Path: " + lfr);
    const std::string fast_path = compressCardinalForwards(lfr);
    LOG_INFO("Explored Cardinal Fast Path: " + fast_path);
    const bool ok = executeFastSequence(mouse, fast_path, start_from_back_wall);
    maze_mouse->setMazeMask(previous_mask);
    return ok;
}

bool traverseExploredDiagonalPath(Mouse* mouse, MazeMouse* maze_mouse,
                                  const std::vector<std::array<int, 2>>& goals,
                                  bool                                   start_from_back_wall)
{
    if (mouse == nullptr || maze_mouse == nullptr)
        return false;

    if (atAnyGoal(maze_mouse, goals))
    {
        LOG_INFO("Already at goal; no speed-run path needed.");
        return true;
    }
    if (!speedRunYawPreflight(mouse, maze_mouse))
        return false;

    MotionSequenceGuard motion_sequence(mouse);
    const MazeMask      previous_mask = maze_mouse->mazeMask();
    maze_mouse->setMazeMask(MASK_CLOSED);

    std::vector<Cell*> best_path = bestExploredPath(maze_mouse, goals);
    if (best_path.empty())
    {
        maze_mouse->setMazeMask(previous_mask);
        LOG_ERROR("No explored-only diagonal speed run path available.");
        return false;
    }

    colorPath(mouse, best_path);

    std::string lfr = PathConverter::buildLFR(maze_mouse->currentCell(),
                                              maze_mouse->currentDirectionArray(), best_path);
    LOG_INFO("Explored A* LFR Path: " + lfr);

    std::string diag = Diagonalizer::diagonalize(lfr);
    LOG_INFO("Explored Diagonalized Path: " + diag);
    const std::string fast_diag = compressCardinalForwards(diag);
    LOG_INFO("Explored Diagonal Fast Path: " + fast_diag);
    const bool ok = executeFastSequence(mouse, fast_diag, start_from_back_wall);
    maze_mouse->setMazeMask(previous_mask);
    return ok;
}

void detectWalls(Mouse& mouse, MazeMouse& maze_mouse)
{
    Cell* cell = maze_mouse.currentCell();

    const std::string front = maze_mouse.directionAsString(maze_mouse.currentDirectionArray());
    const std::string left  = maze_mouse.directionLeft();
    const std::string right = maze_mouse.directionRight();

    cell->updateWallState(front[0], mouse.wallFront() ? WALL : EXIT);
    cell->updateWallState(left[0], mouse.wallLeft() ? WALL : EXIT);
    cell->updateWallState(right[0], mouse.wallRight() ? WALL : EXIT);
    cell->markExplored();
}

void colorPath(Mouse* mouse, const std::vector<Cell*>& path)
{
    char color = mouse->phaseColor();
    for (Cell* cell : path)
    {
        mouse->setColor(cell->x(), cell->y(), color);
    }
}

} // namespace PathUtils
