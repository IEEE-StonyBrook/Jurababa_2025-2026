// 17,18,19,20
// 6,7,8,9
#include <stdio.h>

#include "../Include/Common/LogSystem.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Navigation/FrontierBasedSearchSolver.h"
#include "../Include/Platform/Simulator/API.h"

#ifdef USING_ROBOT
#include "hardware/uart.h"
#include "pico/stdlib.h"
#endif

void interpretLFRPath(API* apiPtr, std::string lfrPath);
void detectWalls(API& api, InternalMouse& internalMouse);

int main() {
  const bool RUN_ON_SIMULATOR = true;

  // Universal objects
  LogSystem logSystem;
  std::array<int, 2> startCell = {0, 0};
  std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

  // Mouse logic objects
  MazeGraph maze(16, 16);
  InternalMouse mouse(startCell, std::string("n"), goalCells, &maze,
                      &logSystem);
  API api(&mouse, RUN_ON_SIMULATOR);
  api.setUp(startCell, goalCells);
  api.printMaze();

  FrontierBasedSearchSolver frontierSolver(&api, &mouse, true);
  frontierSolver.exploreMaze();

#ifdef USING_ROBOT
  stdio_init_all();
  // Robot objects
  Encoder leftMotorEncoder(20);
  Encoder rightMotorEncoder(7);
  Motor leftMotor(18, 19, &leftMotorEncoder, true);
  Motor rightMotor(6, 7, &rightMotorEncoder);
  ToF leftToF(11, 'L');
  ToF frontToF(12, 'F');
  ToF rightToF(13, 'R');
  IMU imu(5);
  Drivetrain robotDrivetrain(&leftMotor, &rightMotor, &leftToF, &frontToF,
                             &rightToF, &imu);
  API api(&drivetrain, &api, RUN_ON_SIMULATOR);
#endif
  // Maze logic objects
  AStarSolver aStar(&mouse);
  // std::string path = aStar.go(goalCells, true, true);
  // LOG_DEBUG(path);
  // interpretLFRPath(&api, path);

  // while (true) {
  //   LOG_WARNING(aStar.go({{8, 8}}, false, false));
  // }
  return 0;
}

void interpretLFRPath(API* apiPtr, std::string lfrPath) {
  std::stringstream ss(lfrPath);
  std::string token;

  // Seperate into tokens.
  std::vector<std::string> tokens;
  while (std::getline(ss, token, '#')) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }

  // Go through each token and run movement.
  for (std::string t : tokens) {
    if (t == "R") {
      apiPtr->turnRight90();
    } else if (t == "L") {
      apiPtr->turnLeft90();
    } else if (t == "F") {
      apiPtr->moveForward();
    } else if (t == "R45") {
      apiPtr->turnRight45();
    } else if (t == "L45") {
      apiPtr->turnLeft45();
    } else if (t == "FH") {
      apiPtr->moveForwardHalf();
    } else {
      LOG_ERROR("Main.cpp: Unknown token: " + t);
    }
  }
}

bool goIterativeAStar(API* apiPtr, InternalMouse* mouse,
                      AStarSolver* aStar,
                      std::vector<std::array<int, 2>> goalCells,
                      bool diagonalsAllowed = true,
                      bool passThroughGoalCells = false) {
    MazeNode* currNode = mouse->getCurrentRobotNode();

    while (true) {
        // 1. Goal check
        if (mouse->isAGoalCell(currNode)) {
            LOG_INFO("Reached goal at (" +
                     std::to_string(currNode->getCellXPos()) + "," +
                     std::to_string(currNode->getCellYPos()) + ")");
            return true;
        }

        // 2. Get path from A*
        std::string lfrPath =
            aStar->go(goalCells, diagonalsAllowed, passThroughGoalCells);

        if (lfrPath.empty()) {
            LOG_ERROR("No path found!");
            return false;
        }
        LOG_INFO("A* path: " + lfrPath);

        // 3. Diagonalize if allowed
        if (diagonalsAllowed) {
            lfrPath = Diagonalizer::diagonalize(lfrPath);
            LOG_INFO("Diagonalized path: " + lfrPath);
        }

        // 4. Take only the first move
        std::stringstream ss(lfrPath);
        std::string move;
        if (!std::getline(ss, move, '#') || move.empty()) {
            LOG_ERROR("Bad path token!");
            return false;
        }

        // 5. Execute the move
        apiPtr->executeSequence(move);

        // 6. Update exploration
        currNode = mouse->getCurrentRobotNode();
        if (!currNode->getCellIsExplored()) {
            LOG_DEBUG("[RE-CALC] New unexplored node at (" +
                      std::to_string(currNode->getCellXPos()) + "," +
                      std::to_string(currNode->getCellYPos()) + ")");
            detectWalls(*apiPtr, *mouse);
            currNode->markAsExplored();
        }
    }
}

void detectWalls(API& api, InternalMouse& internalMouse) {
    // Check and set walls based on the internal mouse's perception
    if (internalMouse.getCurrentRobotNode()->getIsWall('n')) {
        api.setWall(internalMouse.getCurrentRobotNode()->getCellXPos(),
                    internalMouse.getCurrentRobotNode()->getCellYPos(), "n");
    }
    if (internalMouse.getCurrentRobotNode()->getIsWall('e')) {
        api.setWall(internalMouse.getCurrentRobotNode()->getCellXPos(),
                    internalMouse.getCurrentRobotNode()->getCellYPos(), "e");
    }
    if (internalMouse.getCurrentRobotNode()->getIsWall('s')) {
        api.setWall(internalMouse.getCurrentRobotNode()->getCellXPos(),
                    internalMouse.getCurrentRobotNode()->getCellYPos(), "s");
    }
    if (internalMouse.getCurrentRobotNode()->getIsWall('w')) {
        api.setWall(internalMouse.getCurrentRobotNode()->getCellXPos(),
                    internalMouse.getCurrentRobotNode()->getCellYPos(), "w");
    }
}