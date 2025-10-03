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
  // Sensors
    Encoder leftEncoder(20);
    Encoder rightEncoder(7);
    ToF leftToF(11, 'L');
    ToF frontToF(12, 'F');
    ToF rightToF(13, 'R');
    IMU imu(5);

    // Motors
    Motor leftMotor(18, 19, &leftEncoder, true);
    Motor rightMotor(6, 7, &rightEncoder);
    Drivetrain robotDrivetrain(&leftMotor, &rightMotor, &leftToF, &frontToF, &rightToF, &imu);
    // LOG_DEBUG("CORE1 Init")
    // Signal Core0 that Core1 is ready
    multicore_fifo_push_blocking(1);

    // Optional: set initial velocity
    // leftMotor.setUpPIDControllerWithFeedforward(5.0f, 0.00677f, 0.000675f, 0.0f, 0.0f);
    // leftMotor.setContinuousDesiredMotorVelocityMMPerSec(100.0f);
    // LOG_DEBUG("CORE1 Loop")
    while (true) {
        // Read sensors
        // LOG_DEBUG("CORE1 Read")
        MulticoreSensorData local{};
        local.left_encoder_count = leftEncoder.getCurrentEncoderTickCount();
        local.right_encoder_count = rightEncoder.getCurrentEncoderTickCount();
        local.tof_left_mm = static_cast<int16_t>(leftToF.getToFDistanceFromWallMM());
        local.tof_front_mm = static_cast<int16_t>(frontToF.getToFDistanceFromWallMM());
        local.tof_right_mm = static_cast<int16_t>(rightToF.getToFDistanceFromWallMM());
        local.imu_yaw = imu.getIMUYawDegreesNeg180ToPos180();
        local.timestamp_ms = to_ms_since_boot(get_absolute_time());

        // Publish sensor snapshot to Core0
        MulticoreSensorHub::publish(local);
        // MulticoreSensorData test{};
        // MulticoreSensorHub::snapshot(test);
        // LOG_DEBUG("Left encoder CORE1: " + std::to_string(local.left_encoder_count));
        // LOG_DEBUG("Front ToF CORE1: " + std::to_string(local.tof_front_mm));
        // LOG_DEBUG("Left TEST encoder CORE1: " + std::to_string(test.left_encoder_count));
        // LOG_DEBUG("Front TEST ToF CORE1: " + std::to_string(test.tof_front_mm));

        sleep_ms(250);
    }
}

int main() {
  stdio_init_all();
  sleep_ms(3000);

  MulticoreSensorHub::init();
  multicore_launch_core1(core1_publisher);

  // Wait until Core1 signals it finished initializing its sensors
  multicore_fifo_pop_blocking();

  // LOG_DEBUG("Test")

  // Maze / planning objects
  std::array<int, 2> startCell = {0, 0};
  std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
  MazeGraph maze(16, 16);
  InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);

  // Maze logic objects
  // AStarSolver aStar(&mouse);
  // std::string path = aStar.go(goalCells, true, true);
  // LOG_DEBUG(path);
  // interpretLFRPath(&api, path);
  // // while (true) {
  // //   LOG_WARNING(aStar.go({{8, 8}}, false, false));
  // // }
  // LOG_DEBUG("Initialize")
  // Main loop: high-level planning, sensor reads, etc.
  while (true) {
    // LOG_DEBUG("Loop")
      MulticoreSensorData sensors;
      MulticoreSensorHub::snapshot(sensors); // lock-free read

      // Example: print sensor values or feed into planner
      // LOG_DEBUG("Left encoder: " + std::to_string(sensors.left_encoder_count));
      // LOG_DEBUG("Front ToF: " + std::to_string(sensors.tof_front_mm));
      // LOG_DEBUG("IMU_YAW: " + std::to_string(sensors.imu_yaw));

      sleep_ms(250);
  }
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