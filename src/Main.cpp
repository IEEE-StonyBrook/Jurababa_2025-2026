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
  Encoder rightMotorEncoder(8);
  Motor leftMotor(18, 19, &leftMotorEncoder, true);
  Motor rightMotor(6, 7, &rightMotorEncoder);
  ToF leftToF(11, 'L');
  ToF frontToF(12, 'F');
  ToF rightToF(13, 'R');
  IMU imu(5);
  Drivetrain robotDrivetrain(&leftMotor, &rightMotor, &leftToF, &frontToF,
                             &rightToF, &imu);
  // API api(&robotDrivetrain, &mouse);
  // api.setUp(startCell, goalCells);
  // api.printMaze();

  LOG_DEBUG("Configuring motors...");

  // Tune values: start conservative, adjust later
  leftMotor.configurePIDWithFF(0.005f, 0.0f, 0.0f, 0.00136727f, 0.252653f);
  rightMotor.configurePIDWithFF(0.005f, 0.0f, 0.0f, 0.0012421f, 0.393111f);

  leftMotor.setDesiredVelocityMMPerSec(200.0f);
  rightMotor.setDesiredVelocityMMPerSec(200.0f);
  // Set target velocities (mm/s)
  // leftMotor.setDesiredVelocityMMPerSec(100.0f);
  // rightMotor.setDesiredVelocityMMPerSec(100.0f);

  LOG_DEBUG("Starting velocity test...");

  while (1) {
    leftMotor.controlTick();
    rightMotor.controlTick();
    LOG_DEBUG(
        "Left vel: " + std::to_string(leftMotor.getWheelVelocityMMPerSec()) +
        " | Right vel: " +
        std::to_string(rightMotor.getWheelVelocityMMPerSec()));
  }
  // const float step = 0.05f;  // smaller = more resolution
  // for (float pwm = 0.0f; pwm <= 1.0f; pwm += step) {
  //   LOG_DEBUG("Applying PWM: " + std::to_string(pwm));

  //   leftMotor.applyPWM(pwm);
  //   rightMotor.applyPWM(pwm);

  //   // Let motors settle (2 seconds)
  //   uint64_t startTime = time_us_64();
  //   while ((time_us_64() - startTime) < 10000000) {  // run for 10 seconds
  //   (10,000,000 microseconds)
  //     leftMotor.controlTick();
  //     rightMotor.controlTick();
  //   }

  //   // Record measured steady-state velocity
  //   float leftVel = leftMotor.getWheelVelocityMMPerSec();
  //   float rightVel = rightMotor.getWheelVelocityMMPerSec();
  //   float rightPos = rightMotor.getWheelPositionMM();

  //   LOG_DEBUG("PWM: " + std::to_string(pwm) +
  //             " | LeftVel: " + std::to_string(leftVel) +
  //             " | RightVel: " + std::to_string(rightVel) +
  //             " | RightPos: " + std::to_string(rightPos));

  //   // Short pause before next increment
  //   sleep_ms(500);
  // }

  leftMotor.stopMotor();
  rightMotor.stopMotor();

  LOG_DEBUG("Sweep complete. Export log -> Excel or Python for analysis.");
  while (true) {
    sleep_ms(1000);
  }
  // while (true) {
  //   leftMotor.controlTick();
  //   // rightMotor.controlTick();

  //   // Log every ~100ms instead of every 10ms to reduce spam
  //   static int counter = 0;
  //   if (++counter >= 10) {
  //     LOG_DEBUG("Left vel: " +
  //     std::to_string(leftMotor.getWheelVelocityMMPerSec())); counter = 0;
  //   }

  //   sleep_ms(10);
  // }

  // Maze logic objects
  // AStarSolver aStar(&mouse);
  // std::string path = aStar.go(goalCells, true, true);
  // LOG_DEBUG(path);
  // interpretLFRPath(&api, path);
  // // while (true) {
  // //   LOG_WARNING(aStar.go({{8, 8}}, false, false));
  // // }
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