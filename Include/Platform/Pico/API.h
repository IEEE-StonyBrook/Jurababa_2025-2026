#ifndef API_H
#define API_H

#include <array>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Platform/IAPIInterface.h"
#include "Platform/Pico/Config.h"
#include "Maze/InternalMouse.h"
#include "Platform/Pico/CommandHub.h"
#include "Platform/Pico/Robot/Drivetrain.h"

class API : public IAPIInterface
{
  public:
    API(InternalMouse* internalMouse);

    // Maze dimensions (override from IAPIInterface).
    int mazeWidth() override;
    int mazeHeight() override;

    // Wall queries (override from IAPIInterface).
    bool wallLeft() override;
    bool wallFront() override;
    bool wallRight() override;

    // Movement (override from IAPIInterface).
    void moveForwardHalf() override;
    void moveForward() override;
    void moveForward(int steps) override;
    void turnLeft45() override;
    void turnLeft90() override;
    void turnRight45() override;
    void turnRight90() override;
    void turn(int degrees) override;

    // Execute command sequence like "L90#F5#R45" (override from IAPIInterface).
    void executeSequence(const std::string& sequence) override;

    // Maze walls (override from IAPIInterface).
    void setWall(int x, int y, const std::string& dir) override;
    void clearWall(int x, int y, const std::string& dir) override;

    // Cell coloring and labels (override from IAPIInterface).
    void setColor(int x, int y, char color) override;
    void clearColor(int x, int y) override;
    void clearAllColor() override;
    void setText(int x, int y, const std::string& text) override;
    void clearText(int x, int y) override;
    void clearAllText() override;

    // Setup and print (override from IAPIInterface).
    void setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells) override;
    void printMaze() override;

    // Pico-specific methods (not in interface).
    void        goToCenterFromEdge();
    std::string printMazeRow(int row);

    // Simulator IO.
    std::string getSimulatorResponse(std::string cmd);
    int         getSimulatorIntegerResponse(std::string cmd);
    bool        getSimulatorBoolResponse(std::string cmd);

    bool runOnSimulator;

  private:
    Drivetrain*    drivetrain;
    InternalMouse* internalMouse;
};

#endif