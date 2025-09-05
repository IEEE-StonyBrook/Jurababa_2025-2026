#include "API.h"

#include <iostream>
#include <sstream>

API::API(InternalMouse* internalMouse, bool runOnSimulator)
    : internalMouse(internalMouse), runOnSimulator(runOnSimulator) {}

#ifdef USING_ROBOT
API::API(Drivetrain* drivetrain, InternalMouse* internalMouse,
         bool runOnSimulator)
    : drivetrain(drivetrain),
      internalMouse(internalMouse),
      runOnSimulator(runOnSimulator) {}
#endif

int API::mazeWidth() { return internalMouse->getMazeWidth(); }

int API::mazeHeight() { return internalMouse->getMazeHeight(); }

bool API::wallLeft() {
  if (runOnSimulator) return getSimulatorBoolResponse("wallLeft");
  return false;
}
bool API::wallFront() {
  if (runOnSimulator) return getSimulatorBoolResponse("wallFront");
  return false;
}
bool API::wallRight() {
  if (runOnSimulator) return getSimulatorBoolResponse("wallFront");
  return false;
}

void API::moveForwardHalf() {
  if (runOnSimulator) getSimulatorResponse("moveForwardHalf");
}
void API::moveForward() {
  if (runOnSimulator) getSimulatorResponse("moveForward");
  internalMouse->moveIMForwardOneCell(1);
}

void API::moveForward(int steps) {
  if (runOnSimulator) {
    std::ostringstream commandStream;
    commandStream << "moveForward" << steps;

    getSimulatorResponse("moveForwardHalf");
  }
  internalMouse->moveIMForwardOneCell(steps);
}

void API::turnLeft45() {
  if (runOnSimulator) getSimulatorResponse("turnLeft45");
  internalMouse->turnIM45DegreeStepsRight(-1);
}
void API::turnLeft90() {
  if (runOnSimulator) getSimulatorResponse("turnLeft");
  internalMouse->turnIM45DegreeStepsRight(-2);
}
void API::turnRight45() {
  if (runOnSimulator) getSimulatorResponse("turnRight45");
  internalMouse->turnIM45DegreeStepsRight(1);
}
void API::turnRight90() {
  if (runOnSimulator) getSimulatorResponse("turnRight");
  internalMouse->turnIM45DegreeStepsRight(2);
}
void API::turn(int degreesDivisibleBy45) {
  int turnsNeeded = (int)(degreesDivisibleBy45 / 45);
  if (runOnSimulator) {
    for (int i = 0; i < abs(turnsNeeded); i++) {
      if (degreesDivisibleBy45 > 0)
        turnRight45();
      else
        turnLeft45();
    }
  }

  internalMouse->turnIM45DegreeStepsRight(turnsNeeded);
}

void API::setWall(int x, int y, const std::string& direction) {
  bool isFourCardinal = direction == "n" || direction == "e" ||
                        direction == "s" || direction == "w";

  bool isNonFourCardinal = direction == "ne" || direction == "se" ||
                           direction == "sw" || direction == "nw";
  if (runOnSimulator) {
    if (isFourCardinal) {
      std::cout << "setWall " << x << " " << y << " " << direction << '\n';
    } else if (isNonFourCardinal) {
      std::cout << "setWall " << x << " " << y << " " << direction[0] << '\n';
      std::cout << "setWall " << x << " " << y << " " << direction[1] << '\n';
    }
  }

  if (isFourCardinal)
    internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y),
                                     direction[0]);
  else if (isNonFourCardinal) {
    internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y),
                                     direction[0]);
    internalMouse->setWallExistsNESW(internalMouse->getNodeAtPos(x, y),
                                     direction[1]);
  }
}
void API::clearWall(int x, int y, const std::string& direction) {
  if (runOnSimulator)
    std::cout << "clearWall " << x << " " << y << " " << direction << '\n';

  // FIXME: Add support for next few methods for internal mouse after everything
  // works. Don't overcomplicate now.
}

void API::setColor(int x, int y, char color) {
  if (runOnSimulator)
    std::cout << "setColor " << x << " " << y << " " << color << '\n';
}
void API::clearColor(int x, int y) {
  if (runOnSimulator) std::cout << "clearColor " << x << " " << y << '\n';
}
void API::clearAllColor() {
  if (runOnSimulator) std::cout << "clearAllColor" << '\n';
}

void API::setText(int x, int y, const std::string& text) {
  if (runOnSimulator)
    std::cout << "setText " << x << " " << y << " " << text << '\n';
}
void API::clearText(int x, int y) {
  if (runOnSimulator) std::cout << "clearText " << x << " " << y << '\n';
}
void API::clearAllText() {
  if (runOnSimulator) std::cout << "clearAllText" << '\n';
}

std::string API::getSimulatorResponse(std::string commandUsed) {
  std::cout << commandUsed << '\n';
  std::string simulatorResponse;
  std::getline(std::cin, simulatorResponse);

  return simulatorResponse;
}

int API::getSimulatorIntegerResponse(std::string commmandUsed) {
  return std::stoi(getSimulatorResponse(commmandUsed));
}

bool API::getSimulatorBoolResponse(std::string commandUsed) {
  return getSimulatorResponse(commandUsed) == "true";
}

void API::setUp(std::array<int, 2> startCell,
                std::vector<std::array<int, 2>> goalCells) {
  clearAllColor();
  clearAllText();

  // Adds boundary mazes.
  for (int i = 0; i < internalMouse->getMazeWidth(); i++) {
    setWall(i, 0, "s");
    setWall(i, internalMouse->getMazeHeight() - 1, "n");
  }
  for (int j = 0; j < internalMouse->getMazeHeight(); j++) {
    setWall(0, j, "w");
    setWall(internalMouse->getMazeWidth() - 1, j, "e");
  }

  // Adds grid labels.
  bool SHOW_GRID = true;
  if (SHOW_GRID) {
    for (int i = 0; i < internalMouse->getMazeWidth(); i++) {
      for (int j = 0; j < internalMouse->getMazeHeight(); j++) {
        setText(i, j, std::to_string(i) + "," + std::to_string(j));
      }
    }
  }

  LOG_DEBUG("Starting Ratawoulfie...");

  // Adds color/text to start and goal cells.
  setColor(startCell[0], startCell[1], 'B');
  setText(startCell[0], startCell[1], "Start");

  for (const auto& goalCell : goalCells) {
    setColor(goalCell[0], goalCell[1], 'G');
    setText(goalCell[0], goalCell[1], "End");
  }
}