#include "Navigation/FrontierBasedSearchSolver.h"

#include <algorithm>
#include <unordered_set>

#include "Navigation/PathUtils.h"

// Static member definition
int FrontierBased::distanceGrid_[16][16];

/**
 * @brief Initializes the distance grid with infinity.
 */
void FrontierBased::initializeDistanceGrid(InternalMouse& mouse)
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
void FrontierBased::updateDistances(InternalMouse& mouse, bool diagonalsAllowed)
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
 * @brief Gets the best neighbor to move to.
 *
 * Prioritizes unexplored cells (they have distance 0).
 * Falls back to explored cells with lowest distance (toward nearest unexplored).
 * Returns nullptr if no frontiers remain (exploration complete).
 */
MazeNode* FrontierBased::getBestNeighbor(InternalMouse& mouse, MazeNode* current,
                                         bool diagonalsAllowed)
{
    MazeNode* bestUnexplored   = nullptr;
    MazeNode* bestExplored     = nullptr;
    int       bestExploredDist = INFINITY_DIST + 1;

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

        if (!neighbor->getCellIsExplored())
        {
            // Found an unexplored neighbor - take it
            bestUnexplored = neighbor;
            break;  // Immediately prefer unexplored
        }
        else
        {
            // Track explored neighbor with lowest distance to frontier
            int nx   = neighbor->getCellXPos();
            int ny   = neighbor->getCellYPos();
            int dist = distanceGrid_[nx][ny];
            if (dist < bestExploredDist)
            {
                bestExploredDist = dist;
                bestExplored     = neighbor;
            }
        }
    }

    // If we found an unexplored neighbor, return it
    if (bestUnexplored)
    {
        return bestUnexplored;
    }

    // If best explored distance is still INFINITY, no frontiers exist - exploration done
    if (bestExploredDist >= INFINITY_DIST)
    {
        return nullptr;
    }

    return bestExplored;
}

/**
 * @brief Updates the distance text display for all cells.
 */
void FrontierBased::updateDistanceDisplay(IAPIInterface& api)
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
void FrontierBased::explore(InternalMouse& mouse, IAPIInterface& api, bool diagonalsAllowed)
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

        // Mark dead-end neighbors (cells with 3 walls)
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
 */
void FrontierBased::moveDirectlyToAdjacent(IAPIInterface& api, InternalMouse& mouse,
                                           MazeNode* target)
{
    MazeNode* current = mouse.getCurrentRobotNode();

    int dx = target->getCellXPos() - current->getCellXPos();
    int dy = target->getCellYPos() - current->getCellYPos();

    std::array<int, 2> currentDir = mouse.getCurrentRobotDirArray();
    std::array<int, 2> targetDir  = {dx, dy};

    if (currentDir == targetDir)
    {
        api.moveForward();
    }
    else
    {
        int cross = currentDir[0] * targetDir[1] - currentDir[1] * targetDir[0];
        int dot   = currentDir[0] * targetDir[0] + currentDir[1] * targetDir[1];

        if (cross > 0)
        {
            if (dot == 0)
            {
                api.turnLeft90();
            }
            else if (dot < 0)
            {
                api.turnLeft90();
                api.turnLeft45();
            }
            else
            {
                api.turnLeft45();
            }
        }
        else if (cross < 0)
        {
            if (dot == 0)
            {
                api.turnRight90();
            }
            else if (dot < 0)
            {
                api.turnRight90();
                api.turnRight45();
            }
            else
            {
                api.turnRight45();
            }
        }
        else
        {
            if (dot < 0)
            {
                api.turnRight90();
                api.turnRight90();
            }
        }
        api.moveForward();
    }
}

/**
 * @brief Cascades dead-end marking from a cell.
 */
void FrontierBased::markDeadEndCascade(InternalMouse& mouse, IAPIInterface& api, MazeNode* cell,
                                       bool diagonalsAllowed)
{
    std::vector<MazeNode*> neighbors = mouse.getNodeNeighbors(cell, false);

    for (MazeNode* neighbor : neighbors)
    {
        if (!neighbor->getCellIsExplored() && neighbor->getWallCount() == 3)
        {
            neighbor->markAsExplored();
            api.setColor(neighbor->getCellXPos(), neighbor->getCellYPos(), api.getPhaseColor());
            api.setText(neighbor->getCellXPos(), neighbor->getCellYPos(), "");
            markDeadEndCascade(mouse, api, neighbor, diagonalsAllowed);
        }
    }
}
