// 17,18,19,20
// 6,7,8,9
#include <stdio.h>

#include "API.h"
#include "Utils/LogSystem.h"
#include "Virtual/Solver/AStarSolver.h"

#ifdef USING_ROBOT
#include "hardware/uart.h"
#include "pico/stdlib.h"
#endif

int main() {
  const bool RUN_ON_SIMULATOR = true;

  // Mouse logic items
  MazeGraph maze(16, 16);
  LogSystem logSystem;
  InternalMouse mouse({0, 0}, std::string("n"),
                      {{7, 7}, {7, 8}, {8, 7}, {8, 8}}, &maze, &logSystem);
  API api(&mouse, RUN_ON_SIMULATOR);

#ifdef USING_ROBOT
  stdio_init_all();
  // Robot items
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
  API api(&drivetrain, &api, RUN_ON_SIMULATOR)
#endif
  // Algorithms to run
  AStarSolver aStar(&api, &mouse);

  // while (true) {
  //   LOG_WARNING(aStar.go({{8, 8}}, false, false));
  // }

  api.moveForward();
  
  return 0;
}