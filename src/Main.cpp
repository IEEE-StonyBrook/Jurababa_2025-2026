// 17,18,19,20
// 6,7,8,9
#include <stdio.h>

#include "../Include/Navigation/FrontierBasedSearchSolver.h"
#include "PathUtils.h"

#ifdef USING_ROBOT
#include "hardware/uart.h"
#include "pico/stdlib.h"
#endif

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

  FrontierBased frontierSolver;
  frontierSolver.explore(mouse, api, true);

  AStarSolver aStar(&mouse);
  std::string lfrPath = aStar.go({startCell}, true, true);
  interpretLFRPath(&api, lfrPath);

  std::string lfrPath2 = aStar.go(goalCells, true, true);
  interpretLFRPath(&api, lfrPath2);


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
  // AStarSolver aStar(&mouse);
  // std::string path = aStar.go(goalCells, true, true);
  // LOG_DEBUG(path);
  // interpretLFRPath(&api, path);

  // while (true) {
  //   LOG_WARNING(aStar.go({{8, 8}}, false, false));
  // }
  return 0;
}
