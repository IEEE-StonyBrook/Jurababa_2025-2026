#include <stdio.h>

#include "../Include/Common/LogSystem.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Platform/Pico/API.h"
#include "../Include/Platform/Pico/MulticoreSensors.h"
#include "../Include/Platform/Pico/Robot/Battery.h"
#include "../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../Include/Platform/Pico/Robot/Encoder.h"
#include "../Include/Platform/Pico/Robot/IMU.h"
#include "../Include/Platform/Pico/Robot/Motion.h"
#include "../Include/Platform/Pico/Robot/Motor.h"
#include "../Include/Platform/Pico/Robot/ToF.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// Forward declarations
void interpretLFRPath(API* apiPtr, std::string lfrPath);

void publishSensors(Encoder* leftEncoder, Encoder* rightEncoder, ToF* leftToF,
                    ToF* frontToF, ToF* rightToF, IMU* imu) {
  MulticoreSensorData local{};
  local.left_encoder_count = leftEncoder->getTickCount();
  local.right_encoder_count = rightEncoder->getTickCount();
  local.tof_left_mm = static_cast<int16_t>(leftToF->getToFDistanceFromWallMM());
  local.tof_front_mm =
      static_cast<int16_t>(frontToF->getToFDistanceFromWallMM());
  local.tof_right_mm =
      static_cast<int16_t>(rightToF->getToFDistanceFromWallMM());
  local.imu_yaw = imu->getIMUYawDegreesNeg180ToPos180();
  local.timestamp_ms = to_ms_since_boot(get_absolute_time());

  MulticoreSensorHub::publish(local);
  LOG_DEBUG("[Sensors] EncL=" + std::to_string(local.left_encoder_count) +
            " EncR=" + std::to_string(local.right_encoder_count) +
            " ToF(L,F,R)=(" + std::to_string(local.tof_left_mm) + ", " +
            std::to_string(local.tof_front_mm) + ", " +
            std::to_string(local.tof_right_mm) + ")" +
            " Yaw=" + std::to_string(local.imu_yaw));
}

void processCommand(Motion* motion) {
  if (CommandHub::hasPendingCommands()) {
    CommandPacket cmd = CommandHub::receiveBlocking();
    LOG_DEBUG("[Command] Received type=" + std::to_string((int)cmd.type) +
              " param=" + std::to_string(cmd.param));

    switch (cmd.type) {
      case CommandType::MOVE_FWD_HALF:
        LOG_DEBUG("[Motion] Forward half-cell");
        motion->forward(HALF_CELL_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Forward half-cell");
        break;
      case CommandType::MOVE_FWD:
        LOG_DEBUG("[Motion] Forward " + std::to_string(cmd.param) + " cells");
        motion->forward(cmd.param * CELL_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Forward " + std::to_string(cmd.param) + " cells");
        break;
      case CommandType::TURN_LEFT:
        LOG_DEBUG("[Motion] Turn Left 90");
        motion->turn(-90.0f, TURN_TOP_SPEED, 0.0f, TURN_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Turn Left 90");
        break;
      case CommandType::TURN_RIGHT:
        LOG_DEBUG("[Motion] Turn Right 90");
        motion->turn(90.0f, TURN_TOP_SPEED, 0.0f, TURN_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Turn Right 90");
        break;
      case CommandType::STOP:
        LOG_DEBUG("[Motion] Stop command received");
        motion->stop();
        break;
      case CommandType::TURN_ARBITRARY:
        LOG_DEBUG("[Motion] Turn " + std::to_string(cmd.param) + " deg");
        motion->turn(cmd.param, TURN_TOP_SPEED, 0.0f, TURN_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Turn " + std::to_string(cmd.param) + " deg");
        break;
      case CommandType::CENTER_FROM_EDGE:
        LOG_DEBUG("[Motion] Center from edge");
        motion->forward(TO_CENTER_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        LOG_DEBUG("[Motion] Completed Centering");
        break;
      default:
        LOG_ERROR("[Command] Unknown type received");
        break;
    }
  }
}

// Core1 publisher loop: read sensors + handle commands
static void core1_publisher() {
  // Sensors
  Encoder leftEncoder(pio0, 20, true);
  Encoder rightEncoder(pio0, 8, false);
  ToF leftToF(11, 'L');
  ToF frontToF(12, 'F');
  ToF rightToF(13, 'R');
  IMU imu(5);
  imu.resetIMUYawToZero();

  // Motors + drivetrain
  Motor leftMotor(18, 19, nullptr, true);
  Motor rightMotor(6, 7, nullptr, false);
  Drivetrain drivetrain(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder,
                        &imu, &leftToF, &frontToF, &rightToF);
  Motion motion(&drivetrain);

  LOG_DEBUG("[Core1] Initialization complete.");
  motion.resetDriveSystem();

  multicore_fifo_push_blocking(1);  // signal ready
  LOG_DEBUG("[Core1] Ready, entering loop.");

  while (true) {
    processCommand(&motion);
    publishSensors(&leftEncoder, &rightEncoder, &leftToF, &frontToF, &rightToF,
                   &imu);
    sleep_ms(250);
  }
}

int main() {
  stdio_init_all();
  sleep_ms(3000);

  LOG_DEBUG("[Main] Starting init...");
  MulticoreSensorHub::init();
  multicore_launch_core1(core1_publisher);
  multicore_fifo_pop_blocking();  // wait for core1
  LOG_DEBUG("[Main] Core1 ready.");

  // Maze setup
  std::array<int, 2> startCell = {0, 0};
  std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
  MazeGraph maze(16, 16);
  InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);
  API api(&mouse);

  LOG_DEBUG("[Main] Maze and API initialized.");

  // Example: execute path
  api.executeSequence("L#");
  // api.executeSequence("R#");

  while (true) {
    MulticoreSensorData sensors;
    MulticoreSensorHub::snapshot(sensors);

    if (CommandHub::hasPendingCommands()) {
      CommandPacket cmd = CommandHub::receiveBlocking();
      if (cmd.type == CommandType::SNAPSHOT) {
        LOG_DEBUG("[Snapshot] Mask=" + std::to_string((uint32_t)(SensorMask)cmd.param));
        if (sensors.tof_left_exist) mouse.setWallExistsLFR('L');
        if (sensors.tof_front_exist) mouse.setWallExistsLFR('F');
        if (sensors.tof_right_exist) mouse.setWallExistsLFR('R');
      }
    }
    sleep_ms(250);
  }
  return 0;
}

void interpretLFRPath(API* apiPtr, std::string lfrPath) {
  std::stringstream ss(lfrPath);
  std::string token;
  std::vector<std::string> tokens;

  while (std::getline(ss, token, '#')) {
    if (!token.empty()) tokens.push_back(token);
  }

  for (std::string t : tokens) {
    LOG_DEBUG("[Interpreter] Token=" + t);
    if (t == "R") apiPtr->turnRight90();
    else if (t == "L") apiPtr->turnLeft90();
    else if (t == "F") apiPtr->moveForward();
    else if (t == "R45") apiPtr->turnRight45();
    else if (t == "L45") apiPtr->turnLeft45();
    else if (t == "FH") apiPtr->moveForwardHalf();
    else LOG_ERROR("[Interpreter] Unknown token: " + t);
  }
}
