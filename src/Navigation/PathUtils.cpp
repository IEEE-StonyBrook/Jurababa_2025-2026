#include "Navigation/PathUtils.h"

#include <sstream>
#include <string>
#include <vector>

#include "Common/LogSystem.h"
#include "Navigation/Diagonalizer.h"

void interpretLFRPath(IAPIInterface* apiPtr, std::string lfrPath)
{
    std::stringstream ss(lfrPath);
    std::string       token;

    // Seperate into tokens.
    std::vector<std::string> tokens;
    while (std::getline(ss, token, '#'))
    {
        if (!token.empty())
        {
            tokens.push_back(token);
        }
    }

    // Go through each token and run movement.
    for (std::string t : tokens)
    {
        if (t == "R")
        {
            apiPtr->arcTurnRight90();  // Use arc turn for smoother motion
        }
        else if (t == "L")
        {
            apiPtr->arcTurnLeft90();  // Use arc turn for smoother motion
        }
        else if (t == "F")
        {
            apiPtr->moveForward();
        }
        else if (t == "R45")
        {
            apiPtr->arcTurnRight45();  // Use arc turn for smoother motion
        }
        else if (t == "L45")
        {
            apiPtr->arcTurnLeft45();  // Use arc turn for smoother motion
        }
        else if (t == "FH")
        {
            apiPtr->moveForwardHalf();
        }
        else
        {
            LOG_ERROR("Main.cpp: Unknown token: " + t);
        }
    }
}

void setAllExplored(InternalMouse* mouse)
{
    int numCols = mouse->getMazeWidth();
    int numRows = mouse->getMazeHeight();
    // Sets all cells to explored.
    for (int i = 0; i < numCols; i++)
    {
        for (int j = 0; j < numRows; j++)
        {
            mouse->getNodeAtPos(i, j)->markAsExplored();
        }
    }
}

bool traversePathIteratively(IAPIInterface* apiPtr, InternalMouse* mouse,
                             std::vector<std::array<int, 2>> goalCells, bool diagonalsAllowed,
                             bool allExplored, bool avoidGoalCells)
{
    MazeNode*   currNode = mouse->getCurrentRobotNode();
    AStarSolver aStar(mouse);

    if (allExplored)
    {
        LOG_INFO("Marking all explored for debug run.");
        setAllExplored(mouse);
    }

    // Save original goal cells to restore after traversal
    std::vector<std::array<int, 2>> originalGoalCells = mouse->getGoalCells();
    mouse->setGoalCells(goalCells);

    while (true)
    {
        // --- Step 1: Exploration marking ---
        currNode->markAsExplored();

        // --- Step 2: Goal check ---
        if (mouse->isAGoalCell(currNode))
        {
            LOG_INFO("Reached goal at (" + std::to_string(currNode->getCellXPos()) + "," +
                     std::to_string(currNode->getCellYPos()) + ")");
            break;
        }

        // --- Step 3: Detect walls ---
        detectWalls(*apiPtr, *mouse);

        // --- Step 4: Get path from A* ---
        std::string lfrPath = aStar.go(goalCells, diagonalsAllowed, !avoidGoalCells);
        LOG_DEBUG("Goal Cells:");
        for (const auto& cell : goalCells)
        {
            LOG_DEBUG(" - (" + std::to_string(cell[0]) + "," + std::to_string(cell[1]) + ")");
        }

        if (lfrPath.empty())
        {
            LOG_ERROR("No path found!");
            mouse->setGoalCells(originalGoalCells);
            return false;
        }

        LOG_INFO("A* LFR Path: " + lfrPath);

        // --- Step 5: Optional diagonalization ---
        if (allExplored && diagonalsAllowed)
        {
            lfrPath = Diagonalizer::diagonalize(lfrPath);
            LOG_INFO("Diagonalized Path: " + lfrPath);
        }

        bool SHOW_PATH       = true;
        char GOAL_PATH_COLOR = 'G';
        // --- Step 6: Optional coloring (like old code) ---
        if (SHOW_PATH && allExplored)
        {
            // Just color current path from A*
            std::stringstream ss(lfrPath);
            std::string       token;
            int               x = currNode->getCellXPos();
            int               y = currNode->getCellYPos();
            while (std::getline(ss, token, '#'))
            {
                apiPtr->setColor(x, y, GOAL_PATH_COLOR);
                // NOTE: For real coloring, you’d need to map tokens back to cells.
            }
        }

        // --- Step 7: Execute path step-by-step ---
        std::stringstream ss(lfrPath);
        std::string       move;
        while (std::getline(ss, move, '#'))
        {
            if (move.empty())
                continue;

            apiPtr->executeSequence(move);

            currNode = mouse->getCurrentRobotNode();

            if (!currNode->getCellIsExplored())
            {
                LOG_DEBUG("[RE-CALC] Hit unexplored cell at (" +
                          std::to_string(currNode->getCellXPos()) + "," +
                          std::to_string(currNode->getCellYPos()) + ")");
                break;
            }

            LOG_DEBUG("[RE-USE] Continuing with path token: " + move);
        }

        LOG_DEBUG("Breaking to re-calc path");
        // break;  // Keep the FIXME break like the original Java version
    }

    mouse->setGoalCells(originalGoalCells);
    return true;
}

void detectWalls(IAPIInterface& api, InternalMouse& internalMouse)
{
    MazeNode* currCell = internalMouse.getCurrentRobotNode();
    if (api.wallFront())
    {
        api.setWall(currCell->getCellXPos(), currCell->getCellYPos(),
                    internalMouse.getDirectionAsString(internalMouse.getCurrentRobotDirArray()));
    }
    if (api.wallLeft())
    {
        api.setWall(currCell->getCellXPos(), currCell->getCellYPos(),
                    internalMouse.getDirectionToTheLeft());
    }
    if (api.wallRight())
    {
        api.setWall(currCell->getCellXPos(), currCell->getCellYPos(),
                    internalMouse.getDirectionToTheRight());
    }
}