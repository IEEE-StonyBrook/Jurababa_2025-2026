#ifndef API_H
#define API_H

#include <array>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../../Include/Platform/Pico/Config.h"
#include "../../Maze/InternalMouse.h"
#include "Robot/Drivetrain.h"
#include "Robot/Motion.h"
#include "CommandHub.h"


class API {
 public:
  API(InternalMouse* internalMouse);

  // Maze dimensions.
  int mazeWidth();
  int mazeHeight();

  // Wall queries.
  bool wallLeft();
  bool wallFront();
  bool wallRight();

  // Movement.
  void moveForwardHalf();
  void moveForward();
  void moveForward(int steps);
  void turnLeft45();
  void turnLeft90();
  void turnRight45();
  void turnRight90();
  void goToCenterFromEdge();
  void turn(int degrees);

  // Execute command sequence like "L90#F5#R45".
  void executeSequence(const std::string& sequence);

  // Maze walls.
  void setWall(int x, int y, const std::string& dir);
  void clearWall(int x, int y, const std::string& dir);

  // Cell coloring and labels.
  void setColor(int x, int y, char color);
  void clearColor(int x, int y);
  void clearAllColor();
  void setText(int x, int y, const std::string& text);
  void clearText(int x, int y);
  void clearAllText();

  // Setup and print.
  void setUp(std::array<int, 2> startCell,
             std::vector<std::array<int, 2>> goalCells);
  void printMaze();
  std::string printMazeRow(int row);

  // Simulator IO.
  std::string getSimulatorResponse(std::string cmd);
  int getSimulatorIntegerResponse(std::string cmd);
  bool getSimulatorBoolResponse(std::string cmd);

  bool runOnSimulator;

 private:
  Drivetrain* drivetrain;
  InternalMouse* internalMouse;
  Motion* motion;
};

#endif