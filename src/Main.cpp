// 17,18,19,20
// 6,7,8,9
#include <stdio.h>

#include "../Include/Common/LogSystem.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Platform/Pico/API.h"
#include "../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../Include/Platform/Pico/Robot/Encoder.h"
#include "../Include/Platform/Pico/Robot/Motor.h"
#include "../Include/Platform/Pico/Robot/ToF.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// #include "../Include/Platform/Simulator/API.h"

void interpretLFRPath(API* apiPtr, std::string lfrPath);

int main() {
  stdio_init_all();
  sleep_ms(3000);

  // Universal objects
  LogSystem logSystem;
  std::array<int, 2> startCell = {0, 0};
  std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

  // Mouse logic objects
  MazeGraph maze(16, 16);
  InternalMouse mouse(startCell, std::string("n"), goalCells, &maze,
                      &logSystem);

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
  API api(&robotDrivetrain, &mouse);

  api.setUp(startCell, goalCells);
  api.printMaze();
  
  LOG_DEBUG("Sending motor velocity request");
  leftMotor.setContinuousDesiredMotorVelocityMMPerSec(50);

  // Maze logic objects
  AStarSolver aStar(&mouse);
  std::string path = aStar.go(goalCells, true, true);
  LOG_DEBUG(path);
  interpretLFRPath(&api, path);
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