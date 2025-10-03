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
#include "../Include/Platform/Pico/MulticoreSensors.h"

// #include "../Include/Platform/Simulator/API.h"

void interpretLFRPath(API* apiPtr, std::string lfrPath);

// Example publisher run on core1: read sensors and publish into hub
static void core1_publisher() {
  // Instantiate sensors locally on core1 (do not share these objects across cores)
  Encoder leftEncoder(20);
  Encoder rightEncoder(7);
  ToF leftToF(11, 'L');
  ToF frontToF(12, 'F');
  ToF rightToF(13, 'R');
  IMU imu(5);

  multicore_fifo_push_blocking(1); // signal core0 we're ready
  while (true) {
    MulticoreSensorData local{};
    // Use real API names from headers
    local.left_encoder_count = leftEncoder.getCurrentEncoderTickCount();
    local.right_encoder_count = rightEncoder.getCurrentEncoderTickCount();
    local.tof_left_mm = static_cast<int16_t>(leftToF.getToFDistanceFromWallMM());
    local.tof_front_mm = static_cast<int16_t>(frontToF.getToFDistanceFromWallMM());
    local.tof_right_mm = static_cast<int16_t>(rightToF.getToFDistanceFromWallMM());
    local.imu_yaw = imu.getIMUYawDegreesNeg180ToPos180();
    // IMU header doesn't expose pitch/roll; leave as zero or add methods if available
    local.timestamp_ms = to_ms_since_boot(get_absolute_time());

    MulticoreSensorHub::publish(local);
    // sampling rate
    sleep_ms(5);
  }
}

int main() {
  stdio_init_all();
  sleep_ms(3000);

  // Initialize the shared hub and launch sensor publisher on core1
  MulticoreSensorHub::init();
  multicore_launch_core1(core1_publisher);

  // Wait until core1 signals it finished initializing its sensors
  uint32_t v = multicore_fifo_pop_blocking();
  (void)v; // value ignored; presence indicates readiness

  // Now initialize hardware that must live on core0 (encoders/motors)

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
  // API api(&robotDrivetrain, &mouse);
  // api.setUp(startCell, goalCells);
  // api.printMaze();

  LOG_DEBUG("Sending motor velocity request");
  leftMotor.setUpPIDControllerWithFeedforward(5.0f, 0.00677f, 0.000675f, 0.0f, 0.0f);
  leftMotor.setContinuousDesiredMotorVelocityMMPerSec(100.0f);

  while(true) {
    // leftMotor.setContinuousDesiredMotorVelocityMMPerSec(50);
    sleep_ms(10);
  }

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