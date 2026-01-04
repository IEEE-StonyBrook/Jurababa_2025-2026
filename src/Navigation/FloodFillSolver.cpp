#include "Navigation/FloodFillSolver.h"

#include <algorithm>
#include <unordered_set>

#include "Navigation/PathUtils.h"

// Static member definition
int FloodFillSolver::distanceGrid_[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief Initializes the distance grid with infinity.
 */
void FloodFillSolver::initializeDistanceGrid(InternalMouse& mouse)
{
    (void)mouse;
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            distanceGrid_[x][y] = INFINITY_DIST;
        }
    }
}

/**
 * @brief Updates distances using wavefront propagation FROM UNEXPLORED FRONTIER CELLS.
 *
 * Frontier cells = unexplored cells adjacent to explored cells (reachable).
 * This creates a distance field where each explored cell knows how far it is
 * from the nearest unexplored frontier. During backtracking, we follow the
 * gradient toward lower distances (toward unexplored cells).
 */
void FloodFillSolver::updateDistances(InternalMouse& mouse, bool diagonalsAllowed)
{
    // Reset all cells to infinity
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            distanceGrid_[x][y] = INFINITY_DIST;
        }
    }

    std::queue<MazeNode*>         queue;
    std::unordered_set<MazeNode*> inQueue;

    // Find all frontier cells (unexplored cells reachable from explored cells)
    // These get distance 0
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            MazeNode* cell = mouse.getNodeAtPos(x, y);
            if (cell == nullptr)
                continue;

            // Skip explored cells and goal cells
            if (cell->getCellIsExplored() || mouse.isAGoalCell(cell))
                continue;

            // Check if this unexplored cell is adjacent to an explored cell
            std::vector<MazeNode*> neighbors = mouse.getNodeNeighbors(cell, diagonalsAllowed);
            for (MazeNode* neighbor : neighbors)
            {
                if (neighbor->getCellIsExplored() &&
                    mouse.getCanMoveBetweenNodes(neighbor, cell, diagonalsAllowed))
                {
                    // This is a frontier cell - reachable unexplored
                    distanceGrid_[x][y] = 0;
                    queue.push(cell);
                    inQueue.insert(cell);
                    break;
                }
            }
        }
    }

    // Wavefront propagation through explored cells
    // Each explored cell gets distance = how far from nearest frontier
    while (!queue.empty())
    {
        MazeNode* cell        = queue.front();
        queue.pop();
        int x           = cell->getCellXPos();
        int y           = cell->getCellYPos();
        int currentDist = distanceGrid_[x][y];

        std::vector<MazeNode*> neighbors = mouse.getNodeNeighbors(cell, diagonalsAllowed);
        for (MazeNode* neighbor : neighbors)
        {
            if (inQueue.find(neighbor) != inQueue.end())
                continue;
            if (!mouse.getCanMoveBetweenNodes(cell, neighbor, diagonalsAllowed))
                continue;

            // Only propagate to/through explored cells
            if (!neighbor->getCellIsExplored())
                continue;

            int nx      = neighbor->getCellXPos();
            int ny      = neighbor->getCellYPos();
            int newDist = currentDist + 1;

            if (newDist < distanceGrid_[nx][ny])
            {
                distanceGrid_[nx][ny] = newDist;
                queue.push(neighbor);
                inQueue.insert(neighbor);
            }
        }
    }
}

/**
 * @brief Calculates turn cost for cardinal directions only.
 *
 * @return 0 for straight, 1 for 90° turn, 2 for 180° turn.
 */
int FloodFillSolver::getTurnCost(InternalMouse& mouse, MazeNode* neighbor)
{
    MazeNode* current = mouse.getCurrentRobotNode();
    int dx = neighbor->getCellXPos() - current->getCellXPos();
    int dy = neighbor->getCellYPos() - current->getCellYPos();

    std::array<int, 2> currentDir = mouse.getCurrentRobotDirArray();

    // Dot product: 1 = same direction (0°), 0 = perpendicular (90°), -1 = opposite (180°)
    int dot = currentDir[0] * dx + currentDir[1] * dy;

    if (dot > 0)
        return 0;  // Straight ahead
    if (dot == 0)
        return 1;  // 90° turn (left or right)
    return 2;      // 180° turn
}

/**
 * @brief Gets the best neighbor to move to.
 *
 * Prioritizes:
 * 1) Unexplored neighbor with least turning required
 * 2) Explored neighbor with lowest (distance + turn_cost) score
 *
 * Returns nullptr if no frontiers remain (exploration complete).
 */
MazeNode* FloodFillSolver::getBestNeighbor(InternalMouse& mouse, MazeNode* current,
                                         bool diagonalsAllowed)
{
    MazeNode* bestUnexplored     = nullptr;
    int       bestUnexploredTurn = 3;  // Higher than max (2)

    MazeNode* bestExplored      = nullptr;
    int       bestExploredScore = INFINITY_DIST + 3;

    std::vector<MazeNode*> neighbors = mouse.getNodeNeighbors(current, diagonalsAllowed);
    for (MazeNode* neighbor : neighbors)
    {
        if (!mouse.getCanMoveBetweenNodes(current, neighbor, diagonalsAllowed))
        {
            continue;
        }
        if (mouse.isAGoalCell(neighbor))
        {
            continue;
        }

        int turnCost = getTurnCost(mouse, neighbor);

        if (!neighbor->getCellIsExplored())
        {
            // For unexplored cells, prefer least turning
            if (turnCost < bestUnexploredTurn)
            {
                bestUnexploredTurn = turnCost;
                bestUnexplored     = neighbor;
            }
        }
        else
        {
            // For explored cells, use combined score: distance + turn_cost
            int nx    = neighbor->getCellXPos();
            int ny    = neighbor->getCellYPos();
            int dist  = distanceGrid_[nx][ny];
            int score = dist + turnCost;

            if (score < bestExploredScore)
            {
                bestExploredScore = score;
                bestExplored      = neighbor;
            }
        }
    }

    // If we found an unexplored neighbor, return it
    if (bestUnexplored)
    {
        return bestUnexplored;
    }

    // If best explored score is still very high, no frontiers exist
    if (bestExploredScore >= INFINITY_DIST)
    {
        return nullptr;
    }

    return bestExplored;
}

/**
 * @brief Updates the distance text display for all cells.
 */
void FloodFillSolver::updateDistanceDisplay(IAPIInterface& api)
{
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        for (int y = 0; y < MAZE_SIZE; y++)
        {
            int dist = distanceGrid_[x][y];
            if (dist < INFINITY_DIST)
            {
                api.setText(x, y, std::to_string(dist));
            }
            else
            {
                api.setText(x, y, "");  // Clear text for unexplored/unreachable cells
            }
        }
    }
}

/**
 * @brief Explores the maze using Flood-Fill algorithm.
 */
void FloodFillSolver::explore(InternalMouse& mouse, IAPIInterface& api, bool diagonalsAllowed)
{
    initializeDistanceGrid(mouse);

    MazeNode* current = mouse.getCurrentRobotNode();

    // Initial setup: detect walls, mark explored
    detectWalls(api, mouse);
    current->markAsExplored();
    api.setColor(current->getCellXPos(), current->getCellYPos(), api.getPhaseColor());

    // Update distances from frontier cells
    updateDistances(mouse, diagonalsAllowed);
    updateDistanceDisplay(api);

    int       moveCount = 0;
    const int MAX_MOVES = 1000;  // Safety limit

    while (moveCount < MAX_MOVES)
    {
        moveCount++;

        // Mark dead-end neighbors (cells with 3 walls are dead-ends)
        markDeadEndCascade(mouse, api, current, diagonalsAllowed);

        // Update distances (frontiers may have changed due to dead-end marking)
        updateDistances(mouse, diagonalsAllowed);
        updateDistanceDisplay(api);

        // Get best neighbor
        MazeNode* next = getBestNeighbor(mouse, current, diagonalsAllowed);

        if (next == nullptr)
        {
            // No more accessible cells - exploration complete
            break;
        }

        // Move to next cell
        moveDirectlyToAdjacent(api, mouse, next);
        current = mouse.getCurrentRobotNode();

        // If this cell is unexplored, detect walls and mark it
        if (!current->getCellIsExplored())
        {
            detectWalls(api, mouse);
            current->markAsExplored();
            api.setColor(current->getCellXPos(), current->getCellYPos(), api.getPhaseColor());
        }
    }

    // Finally visit goal cells
    traversePathIteratively(&api, &mouse, mouse.getGoalCells(), diagonalsAllowed, false, false);
}

/**
 * @brief Moves directly to an adjacent cell without pathfinding.
 *
 * Only supports cardinal directions (N, E, S, W) during exploration.
 */
void FloodFillSolver::moveDirectlyToAdjacent(IAPIInterface& api, InternalMouse& mouse,
                                           MazeNode* target)
{
    MazeNode* current = mouse.getCurrentRobotNode();

    int dx = target->getCellXPos() - current->getCellXPos();
    int dy = target->getCellYPos() - current->getCellYPos();

    std::array<int, 2> currentDir = mouse.getCurrentRobotDirArray();

    // Dot product: 1 = same direction, 0 = perpendicular, -1 = opposite
    int dot   = currentDir[0] * dx + currentDir[1] * dy;
    // Cross product: positive = target is left, negative = target is right
    int cross = currentDir[0] * dy - currentDir[1] * dx;

    if (dot > 0)
    {
        // Straight ahead
        api.moveForward();
    }
    else if (dot < 0)
    {
        // 180° turn
        api.turnRight90();
        api.turnRight90();
        api.moveForward();
    }
    else if (cross > 0)
    {
        // 90° left
        api.turnLeft90();
        api.moveForward();
    }
    else
    {
        // 90° right
        api.turnRight90();
        api.moveForward();
    }
}

/**
 * @brief Cascades dead-end marking from a cell.
 *
 * A cell with 3 walls is a dead-end because the 4th direction is the entry path.
 */
void FloodFillSolver::markDeadEndCascade(InternalMouse& mouse, IAPIInterface& api, MazeNode* cell,
                                       bool diagonalsAllowed)
{
    std::vector<MazeNode*> neighbors = mouse.getNodeNeighbors(cell, false);

    for (MazeNode* neighbor : neighbors)
    {
        // 3 walls = dead-end (4th direction is entry, max walls on a cell is 4)
        if (!neighbor->getCellIsExplored() && neighbor->getWallCount() >= 3)
        {
            neighbor->markAsExplored();
            api.setColor(neighbor->getCellXPos(), neighbor->getCellYPos(), api.getPhaseColor());
            api.setText(neighbor->getCellXPos(), neighbor->getCellYPos(), "");
            markDeadEndCascade(mouse, api, neighbor, diagonalsAllowed);
        }
    }
}
