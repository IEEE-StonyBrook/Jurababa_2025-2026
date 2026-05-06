#include "navigation/path_utils.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "app/api.h"
#include "common/log.h"
#include "common/tof_wall_utils.h"
#include "config/geometry.h"
#include "control/robot.h"
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

std::string wallBits(Cell* cell)
{
    if (cell == nullptr)
        return "N=? E=? S=? W=?";

    return "N=" + std::to_string(cell->hasWall('N')) + " E=" + std::to_string(cell->hasWall('E')) +
           " S=" + std::to_string(cell->hasWall('S')) + " W=" + std::to_string(cell->hasWall('W'));
}

bool readWallSample(API* api, int16_t& left_mm, int16_t& front_mm, int16_t& right_mm);

std::string fixed1(float value)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
    return buffer;
}

std::string cellName(Cell* cell)
{
    if (cell == nullptr)
        return "(?,?)";
    return "(" + std::to_string(cell->x()) + "," + std::to_string(cell->y()) + ")";
}

std::string poseName(bool at_wall_check)
{
    return at_wall_check ? "wall-check" : "center";
}

std::string movementStyleName(API* api)
{
    if (api == nullptr)
        return "?";
    return api->movementStyle() == API::MovementStyle::Smooth ? "smooth" : "stationary";
}

// Sensor data captured once per planning iteration at the wall-check pose.
// Re-used across all tokens emitted from one A* solve.
struct SenseSnapshot
{
    int16_t tof_l_mm  = 0;
    int16_t tof_f_mm  = 0;
    int16_t tof_r_mm  = 0;
    bool    tof_valid = false;
};

std::string headingUpper(const std::string& d)
{
    std::string r = d;
    for (auto& c : r)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

// Robot-relative wall string in {L,F,R} order. '_' = open, letter = wall.
// Cardinal headings only — diagonal headings return "???".
std::string symbolicWalls(Mouse* mouse, Cell* cell)
{
    if (cell == nullptr || mouse == nullptr)
        return "???";

    const std::string dir = mouse->currentDirection();
    if (dir.size() != 1)
        return "???";

    char left_dir = '?', front_dir = '?', right_dir = '?';
    switch (dir[0])
    {
        case 'n':
            left_dir  = 'W';
            front_dir = 'N';
            right_dir = 'E';
            break;
        case 'e':
            left_dir  = 'N';
            front_dir = 'E';
            right_dir = 'S';
            break;
        case 's':
            left_dir  = 'E';
            front_dir = 'S';
            right_dir = 'W';
            break;
        case 'w':
            left_dir  = 'S';
            front_dir = 'W';
            right_dir = 'N';
            break;
        default:
            return "???";
    }

    std::string out = "___";
    if (cell->hasWall(left_dir))
        out[0] = 'L';
    if (cell->hasWall(front_dir))
        out[1] = 'F';
    if (cell->hasWall(right_dir))
        out[2] = 'R';
    return out;
}

std::string tofTriple(const SenseSnapshot& s)
{
    if (!s.tof_valid)
        return "----/----/----";
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%4d/%4d/%4d", s.tof_l_mm, s.tof_f_mm, s.tof_r_mm);
    return buf;
}

// "[NN] @(x,y) H  ToF=L/F/R  walls=LFR" — common prefix for every per-step line.
std::string senseSummary(int step, Mouse* mouse, const SenseSnapshot& s)
{
    Cell*       cell    = mouse != nullptr ? mouse->currentCell() : nullptr;
    std::string heading = mouse != nullptr ? headingUpper(mouse->currentDirection()) : "?";
    char        step_buf[8];
    std::snprintf(step_buf, sizeof(step_buf), "[%2d]", step);
    return std::string(step_buf) + " @" + cellName(cell) + " " + heading + "  ToF=" + tofTriple(s) +
           "  walls=" + symbolicWalls(mouse, cell);
}

std::string goalsList(const std::vector<std::array<int, 2>>& goals)
{
    std::string out;
    for (size_t i = 0; i < goals.size(); ++i)
    {
        if (i > 0)
            out += ",";
        out += "(" + std::to_string(goals[i][0]) + "," + std::to_string(goals[i][1]) + ")";
    }
    return out;
}

void logSearchStartBanner(API* api, const std::vector<std::array<int, 2>>& goals,
                          bool at_wall_check, bool diagonals)
{
    LOG_INFO("=== SEARCH start  goal=" + goalsList(goals) + "  pose=" + poseName(at_wall_check) +
             "  style=" + movementStyleName(api) + "  diagonals=" + std::to_string(diagonals) +
             " ===");
}

void logSearchDoneBanner(int steps)
{
    LOG_INFO("=== SEARCH done  reached goal  steps=" + std::to_string(steps) + " ===");
}

void logSearchFailBanner(const std::string& reason, int step)
{
    LOG_ERROR("=== SEARCH FAILED  " + reason + "  step=" + std::to_string(step) + " ===");
}

void logNoPathDiagnostics(API* api, Mouse* mouse, const SenseSnapshot& snap)
{
    if (mouse == nullptr)
        return;

    Cell* cell = mouse->currentCell();

#ifndef SIMULATOR_BUILD
    if (snap.tof_valid)
    {
        const tof_wall::WallState wall_state =
            tof_wall::evaluate(snap.tof_l_mm, snap.tof_f_mm, snap.tof_r_mm);
        LOG_ERROR("no-path ToF=" + tofTriple(snap) +
                  "  inferred[L,F,R]=" + std::to_string(wall_state.left_wall) + "," +
                  std::to_string(wall_state.front_wall) + "," +
                  std::to_string(wall_state.right_wall) +
                  "  src=" + tof_wall::sourceName(wall_state.source) +
                  "  front_blocked=" + std::to_string(wall_state.front_blocked));
    }
    else
    {
        LOG_ERROR("no-path ToF unavailable");
    }
#else
    (void)snap;
#endif

    const std::string heading   = headingUpper(mouse->currentDirection());
    const std::string api_walls = api != nullptr ? "L=" + std::to_string(api->wallLeft()) +
                                                       " F=" + std::to_string(api->wallFront()) +
                                                       " R=" + std::to_string(api->wallRight())
                                                 : "L=? F=? R=?";
    LOG_ERROR("no-path cell=" + cellName(cell) + " heading=" + heading + "  walls[" +
              wallBits(cell) + "]  api[" + api_walls + "]");
}

bool readWallSample(API* api, int16_t& left_mm, int16_t& front_mm, int16_t& right_mm)
{
    // api->wallSample falls through to live Robot getters on hardware
    // (FirmwareApi::wallSample uses robot()->{left,front,right}Distance()).
    if (api != nullptr && api->wallSample(left_mm, front_mm, right_mm))
        return true;
    return false;
}

void captureSenseSnapshot(API* api, Mouse* mouse, SenseSnapshot& snap)
{
    snap = {};

    if (mouse == nullptr || mouse->currentCell() == nullptr)
        return;

#ifndef SIMULATOR_BUILD
    int16_t left_mm  = 0;
    int16_t front_mm = 0;
    int16_t right_mm = 0;
    if (!readWallSample(api, left_mm, front_mm, right_mm))
        return;

    snap.tof_l_mm  = left_mm;
    snap.tof_f_mm  = front_mm;
    snap.tof_r_mm  = right_mm;
    snap.tof_valid = true;

    const tof_wall::WallState wall_state = tof_wall::evaluate(left_mm, front_mm, right_mm);
    const float               steering_preview =
        wall_state.steering_allowed
            ? tof_wall::steeringAdjustmentDegps(wall_state.side_error_norm, 0.0f)
            : 0.0f;
    Robot*      robot = api != nullptr ? api->robot() : nullptr;
    const float yaw   = robot != nullptr ? robot->angle() : 0.0f;

    // Wall-PD tuning data — hidden at INFO; visible when log priority is raised to DEBUG.
    LOG_DEBUG("SEARCH side  yaw=" + fixed1(yaw) +
              "  src=" + tof_wall::sourceName(wall_state.source) +
              "  err[L,R,chosen]=" + fixed1(wall_state.left_error_norm) + "," +
              fixed1(wall_state.right_error_norm) + "," + fixed1(wall_state.side_error_norm) +
              "  steering_preview_degps=" + fixed1(steering_preview) +
              "  allowed=" + std::to_string(wall_state.steering_allowed) +
              "  front_blocked=" + std::to_string(wall_state.front_blocked));
#else
    (void)api;
#endif
}

void detectWallsFromSample(API* api, Mouse* mouse, SenseSnapshot& snap)
{
    if (api == nullptr || mouse == nullptr)
        return;

    detectWalls(*api, *mouse);
    captureSenseSnapshot(api, mouse, snap);
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
    if (api == nullptr || mouse == nullptr)
        return false;

    MotionSequenceGuard motion_sequence(api);

    Cell*         current = mouse->currentCell();
    AStar         a_star(mouse);
    bool          at_wall_check = start_at_wall_check;
    int           search_step   = 1;
    SenseSnapshot snap;

    logSearchStartBanner(api, goals, at_wall_check, diagonals);

    if (all_explored)
    {
        LOG_INFO("=== Marking all cells explored (debug run) ===");
        setAllExplored(mouse);
    }

    if (at_wall_check)
        api->captureWallSample();

    while (true)
    {
        detectWallsFromSample(api, mouse, snap);
        current->markExplored();

        // Goal check.
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
            LOG_INFO(senseSummary(search_step, mouse, snap) + "  GOAL reached");
            if (at_wall_check)
            {
                LOG_DEBUG("recentre from wall-check, distance_mm=" +
                          fixed1(WALL_CHECK_TO_CENTER_MM));
                if (!api->center_from_wall_check())
                {
                    logSearchFailBanner("recentre at goal failed", search_step);
                    return false;
                }
                at_wall_check = false;
            }
            api->finish_search_move();
            logSearchDoneBanner(search_step);
            break;
        }

        // Plan.
        std::vector<Cell*> cell_path = a_star.cellPath(
            const_cast<std::vector<std::array<int, 2>>&>(goals), diagonals, !avoid_goals);

        if (cell_path.empty())
        {
            LOG_INFO(senseSummary(search_step, mouse, snap) + "  PLAN: no path");
            logNoPathDiagnostics(api, mouse, snap);
            if (at_wall_check)
            {
                LOG_DEBUG("recentre from wall-check, distance_mm=" +
                          fixed1(WALL_CHECK_TO_CENTER_MM));
                if (!api->center_from_wall_check())
                {
                    logSearchFailBanner("recentre after no-path failed", search_step);
                    return false;
                }
                at_wall_check = false;
            }
            api->finish_search_move();
            logSearchFailBanner("no path found", search_step);
            return false;
        }

        colorPath(api, cell_path);

        std::string lfr = PathConverter::buildLFR(mouse->currentCell(),
                                                  mouse->currentDirectionArray(), cell_path);

        // Fully-explored diagonal speed run: blast the whole sequence.
        if (all_explored && diagonals)
        {
            std::string diag = Diagonalizer::diagonalize(lfr);
            LOG_INFO(senseSummary(search_step, mouse, snap) + "  plan=" + diag + "  exec=ALL");
            api->executeSequence(diag);
            logSearchDoneBanner(search_step);
            return true;
        }

        // Step-by-step execution; replan on hitting unexplored territory.
        std::stringstream ss(lfr);
        std::string       move;
        while (std::getline(ss, move, '#'))
        {
            if (move.empty())
                continue;

            const std::string sense_prefix = senseSummary(search_step, mouse, snap);

            if (move == "F")
            {
                LOG_DEBUG(
                    "token F  at_wall_check=" + std::to_string(at_wall_check) + "  total_mm=" +
                    fixed1(at_wall_check ? CELL_SIZE_MM + WALL_CHECK_TO_CENTER_MM : CELL_SIZE_MM));
                bool motion_started =
                    at_wall_check ? api->search_start_from_wall_check() : api->search_advance();
                at_wall_check = true;

                if (!motion_started)
                {
                    logSearchFailBanner("motion start failed", search_step);
                    api->clear_search_move();
                    return false;
                }
                current = mouse->currentCell();
                LOG_INFO(sense_prefix + "  plan=" + lfr + "  exec=F  ->" + cellName(current) + " " +
                         headingUpper(mouse->currentDirection()));
                ++search_step;
                break; // Re-plan after every forward move.
            }

            // Non-F token (turn / arc). Centre first if currently at wall-check pose.
            if (at_wall_check)
            {
                LOG_DEBUG("recentre before token=" + move +
                          ", distance_mm=" + fixed1(WALL_CHECK_TO_CENTER_MM));
                if (!api->center_from_wall_check())
                {
                    logSearchFailBanner("recentre before token failed", search_step);
                    return false;
                }
                at_wall_check = false;
            }

            api->executeSequence(move);
            api->clear_search_move();
            current                   = mouse->currentCell();
            const std::string new_tag = current->explored() ? "" : " (new)";
            LOG_INFO(sense_prefix + "  plan=" + lfr + "  exec=" + move + "  ->" +
                     cellName(current) + " " + headingUpper(mouse->currentDirection()) + new_tag);

            if (!current->explored())
            {
                LOG_DEBUG("hit unexplored cell at " + cellName(current) + " - re-plan");
                ++search_step;
                break;
            }

            LOG_DEBUG("re-using cached path token: " + move);
            ++search_step;
        }

        LOG_DEBUG("breaking outer loop to re-plan");
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
