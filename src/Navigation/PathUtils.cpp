#include "Navigation/PathUtils.h"

#include <sstream>
#include <string>
#include <vector>

#include "Common/LogSystem.h"
#include "Navigation/Diagonalizer.h"
#include "Navigation/PathConverter.h"

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
        // LOG_INFO("Executing token: " + t);
        if (t == "R")
        {
            apiPtr->turnRight90(); // Use regular turn for smoother motion
        }
        else if (t == "L")
        {
            apiPtr->turnLeft90(); // Use regular turn for smoother motion
        }
        else if (t == "F")
        {
            apiPtr->moveForward();
        }
        else if (t == "R45")
        {
            apiPtr->turnRight45(); // Use regular turn for smoother motion
        }
        else if (t == "L45")
        {
            apiPtr->turnLeft45(); // Use regular turn for smoother motion
        }
        else if (t == "FH")
        {
            apiPtr->moveForwardHalf();
        }
        else if (t == "GMF")
        {
            apiPtr->ghostMoveForward(1);
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

    while (true)
    {
        // --- Step 1: Exploration marking ---
        currNode->markAsExplored();

        // --- Step 2: Goal check - check against the target goalCells, not mouse's goal cells
        bool reachedGoal = false;
        for (const auto& gc : goalCells)
        {
            if (currNode->getCellXPos() == gc[0] && currNode->getCellYPos() == gc[1])
            {
                reachedGoal = true;
                break;
            }
        }
        if (reachedGoal)
        {
            // LOG_INFO("Reached goal at (" + std::to_string(currNode->getCellXPos()) + "," +
            //          std::to_string(currNode->getCellYPos()) + ")");
            break;
        }

        // --- Step 3: Detect walls ---
        detectWalls(*apiPtr, *mouse);

        // --- Step 4: Get path from A* ---
        std::vector<MazeNode*> cellPath = aStar.getCellPath(goalCells, diagonalsAllowed, !avoidGoalCells);

        if (cellPath.empty())
        {
            LOG_ERROR("No path found!");
            return false;
        }

        // Color the path cells with the current phase color
        colorPathCells(apiPtr, cellPath);

        // Convert cell path to LFR format
        std::string lfrPath = PathConverter::buildLFRPath(
            mouse->getCurrentRobotNode(),
            mouse->getCurrentRobotDirArray(), cellPath);

        LOG_INFO("A* LFR Path: " + lfrPath);

        // --- Step 5: Optional diagonalization ---
        // When maze is fully explored and diagonals allowed, diagonalize the path
        // and execute it all at once (no re-calculation needed).
        if (allExplored && diagonalsAllowed)
        {
            std::string diagPath = Diagonalizer::diagonalize(lfrPath);
            LOG_INFO("Diagonalized Path: " + diagPath);

            // Execute the entire diagonalized path without interruption
            interpretLFRPath(apiPtr, diagPath);

            return true;
        }

        // --- Step 6: Execute path step-by-step ---
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

void colorPathCells(IAPIInterface* apiPtr, const std::vector<MazeNode*>& cellPath)
{
    char color = apiPtr->getPhaseColor();
    for (MazeNode* node : cellPath)
    {
        apiPtr->setColor(node->getCellXPos(), node->getCellYPos(), color);
    }
}