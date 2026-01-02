#include "Platform/Pico/API.h"

API::API(InternalMouse* internalMouse) : internalMouse(internalMouse), runOnSimulator(false)
{
}

int API::mazeWidth()
{
    return internalMouse->getMazeWidth();
}
int API::mazeHeight()
{
    return internalMouse->getMazeHeight();
}

// Wall detection.
bool API::wallLeft()
{
    return runOnSimulator ? getSimulatorBoolResponse("wallLeft") : false;
}
bool API::wallFront()
{
    return runOnSimulator ? getSimulatorBoolResponse("wallFront") : false;
}
bool API::wallRight()
{
    return runOnSimulator ? getSimulatorBoolResponse("wallRight") : false;
}

// ================== Movement Commands ================== //

void API::moveForwardHalf()
{
    if (runOnSimulator)
        getSimulatorResponse("moveForwardHalf");
    else
        CommandHub::send(CommandType::MOVE_FWD_HALF);
    internalMouse->moveIMForwardOneCell(0.5f); // or handle in core0 with half-cell update
}

void API::moveForward()
{
    if (runOnSimulator)
        getSimulatorResponse("moveForward");
    else
        CommandHub::send(CommandType::MOVE_FWD, 1);
    internalMouse->moveIMForwardOneCell(1);
}

void API::moveForward(int steps)
{
    if (runOnSimulator)
        getSimulatorResponse("moveForward" + std::to_string(steps));
    else
        CommandHub::send(CommandType::MOVE_FWD, steps);
    internalMouse->moveIMForwardOneCell(steps);
}

void API::ghostMoveForward(int steps)
{
    // Ghost move only updates internal mouse position without sending commands.
    internalMouse->moveIMForwardOneCell(steps);
}

void API::goToCenterFromEdge()
{
    CommandHub::send(CommandType::CENTER_FROM_EDGE);
}

void API::turnLeft45()
{
    if (runOnSimulator)
        getSimulatorResponse("turnLeft45");
    else
        LOG_DEBUG("API: Sending TURN_LEFT 45");
    CommandHub::send(CommandType::TURN_LEFT, 1);
    internalMouse->turnIM45DegreeStepsRight(-1);
}

void API::turnLeft90()
{
    if (runOnSimulator)
        getSimulatorResponse("turnLeft90");
    else
        LOG_DEBUG("API: Sending TURN_LEFT 90");
    CommandHub::send(CommandType::TURN_LEFT, 2);
    internalMouse->turnIM45DegreeStepsRight(-2);
}

void API::turnRight45()
{
    if (runOnSimulator)
        getSimulatorResponse("turnRight45");
    else
        LOG_DEBUG("API: Sending TURN_RIGHT 45");
    CommandHub::send(CommandType::TURN_RIGHT, 1);
    internalMouse->turnIM45DegreeStepsRight(1);
}

void API::turnRight90()
{
    if (runOnSimulator)
        getSimulatorResponse("turnRight90");
    else
        LOG_DEBUG("API: Sending TURN_RIGHT 90");
    CommandHub::send(CommandType::TURN_RIGHT, 2);
    internalMouse->turnIM45DegreeStepsRight(2);
}

void API::turn(int degrees)
{
    if (runOnSimulator)
        getSimulatorResponse("turn" + std::to_string(degrees));
    else
        CommandHub::send(CommandType::TURN_ARBITRARY, degrees);
    internalMouse->turnIM45DegreeStepsRight(degrees / 45);
}

// ================== Arc Turns ================== //

void API::arcTurnLeft90()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnLeft90");
    else
        LOG_DEBUG("API: Sending ARC_TURN_LEFT_90");
    CommandHub::send(CommandType::ARC_TURN_LEFT_90);
    internalMouse->turnIM45DegreeStepsRight(-2);
}

void API::arcTurnRight90()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnRight90");
    else
        LOG_DEBUG("API: Sending ARC_TURN_RIGHT_90");
    CommandHub::send(CommandType::ARC_TURN_RIGHT_90);
    internalMouse->turnIM45DegreeStepsRight(2);
}

void API::arcTurnLeft45()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnLeft45");
    else
        LOG_DEBUG("API: Sending ARC_TURN_LEFT_45");
    CommandHub::send(CommandType::ARC_TURN_LEFT_45);
    internalMouse->turnIM45DegreeStepsRight(-1);
}

void API::arcTurnRight45()
{
    if (runOnSimulator)
        getSimulatorResponse("arcTurnRight45");
    else
        LOG_DEBUG("API: Sending ARC_TURN_RIGHT_45");
    CommandHub::send(CommandType::ARC_TURN_RIGHT_45);
    internalMouse->turnIM45DegreeStepsRight(1);
}

// ================== Sequence Execution ================== //

// Parse commands like "F3#L#R90#F2".
void API::executeSequence(const std::string& sequence)
{
    std::istringstream ss(sequence);
    std::string        token;
    while (std::getline(ss, token, '#'))
    {
        if (token.empty())
            continue;

        char cmd   = std::toupper(token[0]);
        int  value = token.size() > 1 ? std::stoi(token.substr(1)) : 0;

        if (cmd == 'F')
            moveForward(value > 0 ? value : 1);
        else if (cmd == 'L')
        {
            LOG_DEBUG("Turning left " + std::to_string(value > 0 ? value : 90) + " degrees");
            turnLeft90();
        }
        else if (cmd == 'R')
        {
            LOG_DEBUG("Turning right " + std::to_string(value > 0 ? value : 90) + " degrees");
            turnRight90();
        }
    }
    // CommandHub::send(CommandType::STOP); // Ensure stop at end
}

// ================== Maze State ================== //

void API::setWall(int x, int y, const std::string& dir)
{
    if (runOnSimulator)
        std::cout << "setWall " << x << " " << y << " " << dir << '\n';
    internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y), dir[0]);
    if (dir.size() > 1)
        internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y), dir[1]);
}

void API::clearWall(int x, int y, const std::string& dir)
{
    if (runOnSimulator)
        std::cout << "clearWall " << x << " " << y << " " << dir << '\n';
}

void API::setColor(int x, int y, char color)
{
    if (runOnSimulator)
        std::cout << "setColor " << x << " " << y << " " << color << '\n';
}

void API::setPhaseColor(char color)
{
    // Store phase color for use during movement updates.
    // This color will be applied to cells as the mouse moves.
    // Note: In simulator mode, this is handled differently.
    // Here we just store it for later use.
    // In movement methods, we will color the current cell.
    phaseColor_ = color;
}

void API::clearColor(int x, int y)
{
    if (runOnSimulator)
        std::cout << "clearColor " << x << " " << y << '\n';
}
void API::clearAllColor()
{
    if (runOnSimulator)
        std::cout << "clearAllColor" << '\n';
}

void API::setText(int x, int y, const std::string& text)
{
    if (runOnSimulator)
        std::cout << "setText " << x << " " << y << " " << text << '\n';
}
void API::clearText(int x, int y)
{
    if (runOnSimulator)
        std::cout << "clearText " << x << " " << y << '\n';
}
void API::clearAllText()
{
    if (runOnSimulator)
        std::cout << "clearAllText" << '\n';
}

// ================== Simulator Helpers ================== //

std::string API::getSimulatorResponse(std::string cmd)
{
    std::cout << cmd << '\n';
    std::string resp;
    std::getline(std::cin, resp);
    return resp;
}

int API::getSimulatorIntegerResponse(std::string cmd)
{
    return std::stoi(getSimulatorResponse(cmd));
}

bool API::getSimulatorBoolResponse(std::string cmd)
{
    return getSimulatorResponse(cmd) == "true";
}

// ================== Maze Setup and Print ================== //

void API::setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells)
{
    clearAllColor();
    clearAllText();

    // Add boundary walls.
    for (int i = 0; i < mazeWidth(); i++)
    {
        setWall(i, 0, "s");
        setWall(i, mazeHeight() - 1, "n");
    }
    for (int j = 0; j < mazeHeight(); j++)
    {
        setWall(0, j, "w");
        setWall(mazeWidth() - 1, j, "e");
    }

    // Add grid labels.
    for (int i = 0; i < mazeWidth(); i++)
    {
        for (int j = 0; j < mazeHeight(); j++)
        {
            setText(i, j, std::to_string(i) + "," + std::to_string(j));
        }
    }

    // Mark start.
    setColor(startCell[0], startCell[1], 'B');
    setText(startCell[0], startCell[1], "Start");

    // Mark goals.
    for (auto& g : goalCells)
    {
        setColor(g[0], g[1], 'G');
        setText(g[0], g[1], "End");
    }
}

void API::printMaze()
{
    std::string maze = "Maze:\n";
    for (int i = 0; i < mazeWidth(); i++)
        maze += "+---";
    maze += "+\n";
    for (int i = mazeHeight() - 1; i >= 0; --i)
    {
        maze += printMazeRow(i) + "\n";
    }
    std::cout << maze << std::endl;
}

std::string API::printMazeRow(int row)
{
    std::string rowStr = "|", eastStr, southStr;
    for (int i = 0; i < mazeWidth(); ++i)
    {
        MazeNode* curr = internalMouse->getNodeAtPos(i, row);
        eastStr += curr->getIsWall('e') ? "   |" : "    ";
        southStr += curr->getIsWall('s') ? "+---" : "+   ";
    }
    return rowStr + eastStr + "\n" + southStr + "+\n";
}