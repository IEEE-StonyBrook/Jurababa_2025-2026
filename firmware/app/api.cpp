#include "app/api.h"

#include <cctype>
#include <iostream>
#include <sstream>
#include <string>

#ifndef SIMULATOR_BUILD
#include "app/commands.h"
#endif
#include "common/log.h"
#include "maze/maze.h"
#include "maze/mouse.h"

API::API(Mouse* mouse) : mouse_(mouse), run_on_simulator(false)
{
}

int API::mazeWidth()
{
    return mouse_->mazeWidth();
}

int API::mazeHeight()
{
    return mouse_->mazeHeight();
}

bool API::wallLeft()
{
    return run_on_simulator ? simulatorBool("wallLeft") : false;
}

bool API::wallFront()
{
    return run_on_simulator ? simulatorBool("wallFront") : false;
}

bool API::wallRight()
{
    return run_on_simulator ? simulatorBool("wallRight") : false;
}

void API::moveForwardHalf()
{
    if (run_on_simulator)
        simulatorResponse("moveForwardHalf");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::MOVE_FWD_HALF);
#endif
    mouse_->moveForward(0.5f);
}

void API::moveForward()
{
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::MOVE_FWD, 1);
#endif
    mouse_->moveForward(1);
}

void API::moveForward(int steps)
{
    // For simulator, call single-step version in a loop (mms doesn't support moveForwardN)
    if (run_on_simulator)
    {
        for (int i = 0; i < steps; i++)
        {
            simulatorResponse("moveForward");
            mouse_->moveForward(1);
        }
        return;
    }
#ifndef SIMULATOR_BUILD
    CommandHub::send(CommandType::MOVE_FWD, steps);
#endif
    mouse_->moveForward(steps);
}

void API::ghostMoveForward(int steps)
{
    // Ghost move only updates internal mouse position
    mouse_->moveForward(steps);
}

void API::goToCenterFromEdge()
{
#ifndef SIMULATOR_BUILD
    CommandHub::send(CommandType::CENTER_FROM_EDGE);
#endif
}

void API::turnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft45");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::TURN_LEFT, 1);
#endif
    mouse_->turn45Steps(-1);
}

void API::turnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::TURN_LEFT, 2);
#endif
    mouse_->turn45Steps(-2);
}

void API::turnRight45()
{
    if (run_on_simulator)
        simulatorResponse("turnRight45");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::TURN_RIGHT, 1);
#endif
    mouse_->turn45Steps(1);
}

void API::turnRight90()
{
    if (run_on_simulator)
        simulatorResponse("turnRight");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::TURN_RIGHT, 2);
#endif
    mouse_->turn45Steps(2);
}

void API::turn(int degrees)
{
    if (run_on_simulator)
        simulatorResponse("turn" + std::to_string(degrees));
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::TURN_ARBITRARY, degrees);
#endif
    mouse_->turn45Steps(degrees / 45);
}

void API::arcTurnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft90");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::ARC_TURN_LEFT_90);
#endif
    mouse_->turn45Steps(-2);
}

void API::arcTurnRight90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight90");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::ARC_TURN_RIGHT_90);
#endif
    mouse_->turn45Steps(2);
}

void API::arcTurnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft45");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::ARC_TURN_LEFT_45);
#endif
    mouse_->turn45Steps(-1);
}

void API::arcTurnRight45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight45");
#ifndef SIMULATOR_BUILD
    else
        CommandHub::send(CommandType::ARC_TURN_RIGHT_45);
#endif
    mouse_->turn45Steps(1);
}

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
}

void API::setWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "setWall " << x << " " << y << " " << dir << '\n';

    Cell* cell = mouse_->cellAt(x, y);
    if (cell)
    {
        mouse_->setWallNESW(cell, dir[0]);
        if (dir.size() > 1)
            mouse_->setWallNESW(cell, dir[1]);
    }
}

void API::clearWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "clearWall " << x << " " << y << " " << dir << '\n';
}

void API::setColor(int x, int y, char color)
{
    if (run_on_simulator)
        std::cout << "setColor " << x << " " << y << " " << color << '\n';
}

void API::setPhaseColor(char color)
{
    phase_color_ = color;
}

char API::phaseColor() const
{
    return phase_color_;
}

void API::clearColor(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearColor " << x << " " << y << '\n';
}

void API::clearAllColor()
{
    if (run_on_simulator)
        std::cout << "clearAllColor" << '\n';
}

void API::setText(int x, int y, const std::string& text)
{
    if (run_on_simulator)
        std::cout << "setText " << x << " " << y << " " << text << '\n';
}

void API::clearText(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearText " << x << " " << y << '\n';
}

void API::clearAllText()
{
    if (run_on_simulator)
        std::cout << "clearAllText" << '\n';
}

std::string API::simulatorResponse(const std::string& cmd)
{
    std::cout << cmd << '\n';
    std::string resp;
    std::getline(std::cin, resp);
    return resp;
}

int API::simulatorInt(const std::string& cmd)
{
    return std::stoi(simulatorResponse(cmd));
}

bool API::simulatorBool(const std::string& cmd)
{
    return simulatorResponse(cmd) == "true";
}

void API::setUp(std::array<int, 2> start, std::vector<std::array<int, 2>> goals)
{
    clearAllColor();
    clearAllText();

    // Add boundary walls
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

    // Add grid labels
    for (int i = 0; i < mazeWidth(); i++)
    {
        for (int j = 0; j < mazeHeight(); j++)
        {
            setText(i, j, std::to_string(i) + "," + std::to_string(j));
        }
    }

    // Mark start
    setColor(start[0], start[1], 'B');
    setText(start[0], start[1], "Start");

    // Mark goals
    for (auto& g : goals)
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
        Cell* curr = mouse_->cellAt(i, row);
        eastStr += curr->hasWall('E') ? "   |" : "    ";
        southStr += curr->hasWall('S') ? "+---" : "+   ";
    }
    return rowStr + eastStr + "\n" + southStr + "+\n";
}
