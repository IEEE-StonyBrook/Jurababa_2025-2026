#include "navigation/flood_fill.h"

#include <algorithm>
#include <unordered_set>

#include "app/api.h"
#include "navigation/path_utils.h"

int FloodFill::distance_grid_[MAZE_SIZE][MAZE_SIZE];

void FloodFill::initDistanceGrid(Mouse& mouse)
{
    (void)mouse;
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            distance_grid_[x][y] = INF_DIST;
        }
    }
}

void FloodFill::updateDistances(Mouse& mouse, bool diagonals)
{
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            distance_grid_[x][y] = INF_DIST;
        }
    }

    std::queue<Cell*> queue;
    std::unordered_set<Cell*> in_queue;

    // Find frontier cells (unexplored non-goal cells reachable from explored)
    // Goal cells are excluded so exploration completes before navigating to goal
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            Cell* cell = mouse.cellAt(x, y);
            if (cell == nullptr)
                continue;

            // Skip explored cells and goal cells
            if (cell->explored() || mouse.isGoal(cell))
                continue;

            for (Cell* neighbor : mouse.cellNeighbors(cell, diagonals))
            {
                if (neighbor->explored() &&
                    mouse.canMoveBetween(neighbor, cell, diagonals))
                {
                    distance_grid_[x][y] = 0;
                    queue.push(cell);
                    in_queue.insert(cell);
                    break;
                }
            }
        }
    }

    // Wavefront propagation through explored cells
    while (!queue.empty())
    {
        Cell* cell = queue.front();
        queue.pop();
        int x = cell->x();
        int y = cell->y();
        int dist = distance_grid_[x][y];

        for (Cell* neighbor : mouse.cellNeighbors(cell, diagonals))
        {
            if (in_queue.find(neighbor) != in_queue.end())
                continue;
            if (!mouse.canMoveBetween(cell, neighbor, diagonals))
                continue;
            if (!neighbor->explored())
                continue;

            int nx = neighbor->x();
            int ny = neighbor->y();
            int new_dist = dist + 1;

            if (new_dist < distance_grid_[nx][ny])
            {
                distance_grid_[nx][ny] = new_dist;
                queue.push(neighbor);
                in_queue.insert(neighbor);
            }
        }
    }
}

int FloodFill::turnCost(Mouse& mouse, Cell* neighbor)
{
    Cell* current = mouse.currentCell();
    int dx = neighbor->x() - current->x();
    int dy = neighbor->y() - current->y();

    std::array<int, 2> dir = mouse.currentDirectionArray();
    int dot = dir[0] * dx + dir[1] * dy;

    if (dot > 0) return 0;  // Straight
    if (dot == 0) return 1; // 90 turn
    return 2;               // 180 turn
}

Cell* FloodFill::bestNeighbor(Mouse& mouse, Cell* current, bool diagonals)
{
    Cell* best_unexplored = nullptr;
    int best_unexplored_turn = 3;

    Cell* best_explored = nullptr;
    int best_explored_score = INF_DIST + 3;

    for (Cell* neighbor : mouse.cellNeighbors(current, diagonals))
    {
        if (!mouse.canMoveBetween(current, neighbor, diagonals))
            continue;

        // Skip goals during exploration (we'll navigate there after)
        if (mouse.isGoal(neighbor))
            continue;

        int turn = turnCost(mouse, neighbor);

        if (!neighbor->explored())
        {
            if (turn < best_unexplored_turn)
            {
                best_unexplored_turn = turn;
                best_unexplored = neighbor;
            }
        }
        else
        {
            int nx = neighbor->x();
            int ny = neighbor->y();
            int dist = distance_grid_[nx][ny];
            int score = dist + turn;

            if (score < best_explored_score)
            {
                best_explored_score = score;
                best_explored = neighbor;
            }
        }
    }

    if (best_unexplored)
        return best_unexplored;

    // If best explored score is still very high, no frontiers exist
    if (best_explored_score >= INF_DIST)
        return nullptr;

    return best_explored;
}

void FloodFill::moveToAdjacent(API& api, Mouse& mouse, Cell* target)
{
    Cell* current = mouse.currentCell();
    int dx = target->x() - current->x();
    int dy = target->y() - current->y();

    std::array<int, 2> dir = mouse.currentDirectionArray();
    int dot = dir[0] * dx + dir[1] * dy;
    int cross = dir[0] * dy - dir[1] * dx;

    if (dot > 0)
    {
        api.moveForward();
    }
    else if (dot < 0)
    {
        api.turnRight90();
        api.turnRight90();
        api.moveForward();
    }
    else if (cross > 0)
    {
        api.turnLeft90();
        api.moveForward();
    }
    else
    {
        api.turnRight90();
        api.moveForward();
    }
}

void FloodFill::markDeadEnds(Mouse& mouse, API& api, Cell* cell, bool diagonals)
{
    for (Cell* neighbor : mouse.cellNeighbors(cell, false))
    {
        if (!neighbor->explored() && neighbor->wallCount() >= 3)
        {
            neighbor->markExplored();
            api.setColor(neighbor->x(), neighbor->y(), api.phaseColor());
            api.setText(neighbor->x(), neighbor->y(), "");
            markDeadEnds(mouse, api, neighbor, diagonals);
        }
    }
}

void FloodFill::updateDisplay(API& api)
{
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            int dist = distance_grid_[x][y];
            if (dist < INF_DIST)
            {
                api.setText(x, y, std::to_string(dist));
            }
            else
            {
                api.setText(x, y, "");
            }
        }
    }
}

void FloodFill::explore(Mouse& mouse, API& api, bool diagonals)
{
    initDistanceGrid(mouse);

    Cell* current = mouse.currentCell();

    PathUtils::detectWalls(api, mouse);
    current->markExplored();
    api.setColor(current->x(), current->y(), api.phaseColor());

    updateDistances(mouse, diagonals);
    updateDisplay(api);

    int moves = 0;
    const int MAX_MOVES = 1000;

    while (moves < MAX_MOVES)
    {
        moves++;

        markDeadEnds(mouse, api, current, diagonals);
        updateDistances(mouse, diagonals);
        updateDisplay(api);

        Cell* next = bestNeighbor(mouse, current, diagonals);

        if (next == nullptr)
            break;

        moveToAdjacent(api, mouse, next);
        current = mouse.currentCell();

        if (!current->explored())
        {
            PathUtils::detectWalls(api, mouse);
            current->markExplored();
            api.setColor(current->x(), current->y(), api.phaseColor());
        }
    }

    PathUtils::traversePath(&api, &mouse, mouse.goals(), diagonals, false, false);
}
