#include "Platform/Simulator/API.h"

#include <iostream>
#include <sstream>

API_SIMULATOR::API_SIMULATOR(InternalMouse* internalMouse, bool runOnSimulator)
    : runOnSimulator(runOnSimulator), internalMouse(internalMouse)
{
}

int API_SIMULATOR::mazeWidth()
{
    return internalMouse->getMazeWidth();
}

int API_SIMULATOR::mazeHeight()
{
    return internalMouse->getMazeHeight();
}

bool API_SIMULATOR::wallLeft()
{
    if (runOnSimulator)
        return getSimulatorBoolResponse("wallLeft");
    return false;
}
bool API_SIMULATOR::wallFront()
{
    if (runOnSimulator)
        return getSimulatorBoolResponse("wallFront");
    return false;
}
bool API_SIMULATOR::wallRight()
{
    if (runOnSimulator)
        return getSimulatorBoolResponse("wallRight");
    return false;
}

void API_SIMULATOR::moveForwardHalf()
{
    if (runOnSimulator)
        getSimulatorResponse("moveForwardHalf");
}
void API_SIMULATOR::moveForward()
{
    if (runOnSimulator)
        getSimulatorResponse("moveForward");
    internalMouse->moveIMForwardOneCell(1);
    int XPos = internalMouse->getCurrentRobotNode()->getCellXPos();
    int YPos = internalMouse->getCurrentRobotNode()->getCellYPos();
    setColor(XPos, YPos, 'y');
}

void API_SIMULATOR::moveForward(int steps)
{
    if (runOnSimulator)
    {
        std::ostringstream commandStream;
        commandStream << "moveForward" << steps;

        getSimulatorResponse("moveForwardHalf");
    }
    internalMouse->moveIMForwardOneCell(steps);
}

void API_SIMULATOR::turnLeft45()
{
    if (runOnSimulator)
        getSimulatorResponse("turnLeft45");
    internalMouse->turnIM45DegreeStepsRight(-1);
}
void API_SIMULATOR::turnLeft90()
{
    if (runOnSimulator)
        getSimulatorResponse("turnLeft");
    internalMouse->turnIM45DegreeStepsRight(-2);
}
void API_SIMULATOR::turnRight45()
{
    if (runOnSimulator)
        getSimulatorResponse("turnRight45");
    internalMouse->turnIM45DegreeStepsRight(1);
}
void API_SIMULATOR::turnRight90()
{
    if (runOnSimulator)
        getSimulatorResponse("turnRight");
    internalMouse->turnIM45DegreeStepsRight(2);
}
void API_SIMULATOR::turn(int degreesDivisibleBy45)
{
    int turnsNeeded = (int)(degreesDivisibleBy45 / 45);
    if (runOnSimulator)
    {
        for (int i = 0; i < abs(turnsNeeded); i++)
        {
            if (degreesDivisibleBy45 > 0)
                turnRight45();
            else
                turnLeft45();
        }
    }

    internalMouse->turnIM45DegreeStepsRight(turnsNeeded);
}

void API_SIMULATOR::arcTurnLeft90()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnLeft90");
    internalMouse->turnIM45DegreeStepsRight(-2);
}

void API_SIMULATOR::arcTurnRight90()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnRight90");
    internalMouse->turnIM45DegreeStepsRight(2);
}

void API_SIMULATOR::arcTurnLeft45()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnLeft45");
    internalMouse->turnIM45DegreeStepsRight(-1);
}

void API_SIMULATOR::arcTurnRight45()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnRight45");
    internalMouse->turnIM45DegreeStepsRight(1);
}

void API_SIMULATOR::executeSequence(const std::string& seq)
{
    std::istringstream ss(seq);
    std::string        command;
    while (ss >> command)
    {
        if (command == "F")
        {
            moveForward();
        }
        else if (command == "FH")
        {
            moveForwardHalf();
        }
        else if (command == "L45")
        {
            turnLeft45();
        }
        else if (command == "L")
        {
            turnLeft90();
        }
        else if (command == "R45")
        {
            turnRight45();
        }
        else if (command == "R")
        {
            turnRight90();
        }
        else
        {
            LOG_WARNING("API.cpp: Unknown command in sequence: " + command);
        }
    }
}

void API_SIMULATOR::setWall(int x, int y, const std::string& direction)
{
    bool isFourCardinal =
        direction == "n" || direction == "e" || direction == "s" || direction == "w";

    bool isNonFourCardinal =
        direction == "ne" || direction == "se" || direction == "sw" || direction == "nw";
    if (runOnSimulator)
    {
        if (isFourCardinal)
        {
            std::cout << "setWall " << x << " " << y << " " << direction << '\n';
        }
        else if (isNonFourCardinal)
        {
            std::cout << "setWall " << x << " " << y << " " << direction[0] << '\n';
            std::cout << "setWall " << x << " " << y << " " << direction[1] << '\n';
        }
    }

    if (isFourCardinal)
        internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y), direction[0]);
    else if (isNonFourCardinal)
    {
        internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y), direction[0]);
        internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y), direction[1]);
    }
}
void API_SIMULATOR::clearWall(int x, int y, const std::string& direction)
{
    if (runOnSimulator)
        std::cout << "clearWall " << x << " " << y << " " << direction << '\n';

    // FIXME: Add support for next few methods for internal mouse after everything
    // works. Don't overcomplicate now.
}

void API_SIMULATOR::setColor(int x, int y, char color)
{
    if (runOnSimulator)
        std::cout << "setColor " << x << " " << y << " " << color << '\n';
}
void API_SIMULATOR::clearColor(int x, int y)
{
    if (runOnSimulator)
        std::cout << "clearColor " << x << " " << y << '\n';
}
void API_SIMULATOR::clearAllColor()
{
    if (runOnSimulator)
        std::cout << "clearAllColor" << '\n';
}

void API_SIMULATOR::setText(int x, int y, const std::string& text)
{
    if (runOnSimulator)
        std::cout << "setText " << x << " " << y << " " << text << '\n';
}
void API_SIMULATOR::clearText(int x, int y)
{
    if (runOnSimulator)
        std::cout << "clearText " << x << " " << y << '\n';
}
void API_SIMULATOR::clearAllText()
{
    if (runOnSimulator)
        std::cout << "clearAllText" << '\n';
}

std::string API_SIMULATOR::getSimulatorResponse(std::string commandUsed)
{
    std::cout << commandUsed << '\n';
    std::string simulatorResponse;
    std::getline(std::cin, simulatorResponse);

    return simulatorResponse;
}

int API_SIMULATOR::getSimulatorIntegerResponse(std::string commmandUsed)
{
    return std::stoi(getSimulatorResponse(commmandUsed));
}

bool API_SIMULATOR::getSimulatorBoolResponse(std::string commandUsed)
{
    return getSimulatorResponse(commandUsed) == "true";
}

void API_SIMULATOR::setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells)
{
    clearAllColor();
    clearAllText();

    // Adds boundary mazes.
    for (int i = 0; i < internalMouse->getMazeWidth(); i++)
    {
        setWall(i, 0, "s");
        setWall(i, internalMouse->getMazeHeight() - 1, "n");
    }
    for (int j = 0; j < internalMouse->getMazeHeight(); j++)
    {
        setWall(0, j, "w");
        setWall(internalMouse->getMazeWidth() - 1, j, "e");
    }

    // Adds grid labels.
    bool SHOW_GRID = true;
    if (SHOW_GRID)
    {
        for (int i = 0; i < internalMouse->getMazeWidth(); i++)
        {
            for (int j = 0; j < internalMouse->getMazeHeight(); j++)
            {
                setText(i, j, std::to_string(i) + "," + std::to_string(j));
            }
        }
    }

    LOG_DEBUG("Starting Ratawoulfie...");

    // Adds color/text to start and goal cells.
    setColor(startCell[0], startCell[1], 'B');
    setText(startCell[0], startCell[1], "Start");

    for (const auto& goalCell : goalCells)
    {
        setColor(goalCell[0], goalCell[1], 'G');
        setText(goalCell[0], goalCell[1], "End");
    }
}

void API_SIMULATOR::printMaze()
{
    std::string mazeString = "Maze:\n";
    for (int i = 0; i < internalMouse->getMazeWidth(); ++i)
    {
        mazeString += "+---";
    }
    mazeString += "+\n";

    for (int i = internalMouse->getMazeHeight() - 1; i >= 0; --i)
    {
        mazeString += printMazeRow(i);
        mazeString += "\n";
    }

    LOG_DEBUG(mazeString);
}

std::string API_SIMULATOR::printMazeRow(int row)
{
    std::string rowString      = "|";
    std::string eastRowString  = "";
    std::string southRowString = "";
    for (int i = 0; i < internalMouse->getMazeWidth(); ++i)
    {
        MazeNode* curr = internalMouse->getNodeAtPos(i, row);
        if (curr->getIsWall('e'))
        {
            eastRowString += "   |";
        }
        else
        {
            eastRowString += "    ";
        }

        if (curr->getIsWall('s'))
        {
            southRowString += "+---";
        }
        else
        {
            southRowString += "+   ";
        }
    }
    rowString += eastRowString;
    rowString += "\n";
    rowString += southRowString;
    rowString += "+\n";

    return rowString;
}