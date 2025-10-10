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


// #include "../Include/Platform/Simulator/API.h"

void interpretLFRPath(API* apiPtr, std::string lfrPath);

void publishSensors(Encoder* leftEncoder, Encoder* rightEncoder, ToF* leftToF,
                    ToF* frontToF, ToF* rightToF, IMU* imu) {
  // Read sensors
  // LOG_DEBUG("CORE1 Read")
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

  // Publish sensor snapshot to Core0
  MulticoreSensorHub::publish(local);
  // MulticoreSensorData test{};
  // MulticoreSensorHub::snapshot(test);
  // LOG_DEBUG("Left encoder CORE1: " +
  // std::to_string(local.left_encoder_count)); LOG_DEBUG("Front ToF CORE1: " +
  // std::to_string(local.tof_front_mm)); LOG_DEBUG("Left TEST encoder CORE1: "
  // + std::to_string(test.left_encoder_count)); LOG_DEBUG("Front TEST ToF
  // CORE1: " + std::to_string(test.tof_front_mm));
}

void processCommand(Motion* motion) {
  if (CommandHub::hasPendingCommands()) {
    CommandPacket cmd = CommandHub::receiveBlocking();
    switch (cmd.type) {
      case CommandType::MOVE_FWD_HALF:
        motion->forward(HALF_CELL_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        break;
      case CommandType::MOVE_FWD:
        motion->forward(cmd.param * CELL_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        break;
      case CommandType::TURN_LEFT:
        motion->spinTurn(-cmd.param, TURN_TOP_SPEED, TURN_ACCEL);
        break;
      case CommandType::TURN_RIGHT:
        motion->spinTurn(cmd.param, TURN_TOP_SPEED, TURN_ACCEL);
        break;
      case CommandType::STOP:
        motion->stop();
        break;
      case CommandType::TURN_ARBITRARY:
        motion->spinTurn(cmd.param, TURN_TOP_SPEED, TURN_ACCEL);
        break;
      case CommandType::CENTER_FROM_EDGE:
        motion->forward(TO_CENTER_DISTANCE_MM, FORWARD_TOP_SPEED,
                        FORWARD_FINAL_SPEED, FORWARD_ACCEL, true);
        break;

      default:
        LOG_ERROR("Unknown command type received.");
        break;
    }
  }
}

// Example publisher run on core1: read sensors and publish into hub
static void core1_publisher() {
  // Sensors
  Encoder leftEncoder(pio0, 20, true);
  Encoder rightEncoder(pio0, 8, false);  // was 7
  ToF leftToF(11, 'L');
  ToF frontToF(12, 'F');
  ToF rightToF(13, 'R');
  IMU imu(5);
  imu.resetIMUYawToZero();

  // Motors
  Motor leftMotor(18, 19, nullptr, true);
  Motor rightMotor(6, 7, nullptr, false);
  Drivetrain drivetrain(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder,
                        &imu, &leftToF, &frontToF, &rightToF);
  Motion motion(&drivetrain);

  LOG_DEBUG("Initialization complete.");
  motion.resetDriveSystem();

  // LOG_DEBUG("CORE1 Init")
  // Signal Core0 that Core1 is ready
  multicore_fifo_push_blocking(1);

  // Optional: set initial velocity
  // leftMotor.setUpPIDControllerWithFeedforward(5.0f, 0.00677f, 0.000675f,
  // 0.0f, 0.0f); leftMotor.setContinuousDesiredMotorVelocityMMPerSec(100.0f);
  // LOG_DEBUG("CORE1 Loop")
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

  MulticoreSensorHub::init();
  multicore_launch_core1(core1_publisher);

  // Wait until Core1 signals it finished initializing its sensors
  multicore_fifo_pop_blocking();

  LOG_DEBUG("Test");

  // Maze / planning objects
  std::array<int, 2> startCell = {0, 0};
  std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
  MazeGraph maze(16, 16);
  InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);
  API api(&mouse);

  // api.goToCenterFromEdge();
  api.executeSequence("L#L#L#");
  // CommandHub::send(CommandType::MOVE_FWD, 1);
  // CommandHub::send(CommandType::TURN_RIGHT, 45);
  // CommandHub::send(CommandType::MOVE_FWD, 2);

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
    MulticoreSensorHub::snapshot(sensors);  // lock-free read

    if (CommandHub::hasPendingCommands()) {
      CommandPacket cmd = CommandHub::receiveBlocking();
      if (cmd.type == CommandType::SNAPSHOT) {
        LOG_DEBUG("Snapshot received with mask: " +
                  std::to_string(static_cast<uint32_t>(static_cast<SensorMask>(cmd.param))));
        if (sensors.tof_left_exist) mouse.setWallExistsLFR('L');
        if (sensors.tof_front_exist) mouse.setWallExistsLFR('F');
        if (sensors.tof_right_exist) mouse.setWallExistsLFR('R');
      }
    }

    // Example: print sensor values or feed into planner
    // LOG_DEBUG("\nLeft encoder: " +
    // std::to_string(sensors.left_encoder_count)); LOG_DEBUG("\nFront ToF: " +
    // std::to_string(sensors.tof_front_mm)); LOG_DEBUG("\nIMU_YAW: " +
    // std::to_string(sensors.imu_yaw));

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

// /**
//  * Runs a PWM sweep on the given motor, applying PWM values from startPWM to
//  * endPWM.
//  *
//  * @param motor Pointer to the motor to test.
//  * @param startPWM Starting PWM value (e.g., 0.0f).
//  * @param endPWM Ending PWM value (e.g., 1.0f or -1.0f).
//  * @param stepPWM Step size for PWM (e.g., 0.05f). Automatically negated if
//  * startPWM > endPWM.
//  * @param settleTimeMs Time in milliseconds to wait at each PWM step for
//  * velocity to stabilize.
//  * @param controlTickPeriodMs Period in milliseconds to call
//  * motor->controlTick() during settling.
//  * @return Vector of FeedforwardSample containing applied PWM and measured
//  * velocity.
//  *
//  * Use printed values and the following calculators:
//  * https://www.desmos.com/calculator/qhiaubdod3 - kS and kV calculator
//  * Do not include 0.0 velocity values!
//  *
//  * Note: Comment Motor::controlTick() applyPWM() call to run open-loop tests.
//  */
// std::vector<FeedforwardSample> runPWMSweep(Motor* motor, float startPWM,
//                                            float endPWM, float stepPWM,
//                                            int settleTimeMs,
//                                            int controlTickPeriodMs) {
//   if (startPWM > endPWM && stepPWM > 0) stepPWM = -stepPWM;

//   std::vector<FeedforwardSample> sweepResults;

//   // Sweep PWM from startPWM to endPWM in steps of stepPWM.
//   for (float pwm = startPWM; (stepPWM > 0 ? pwm <= endPWM : pwm >= endPWM);
//        pwm += stepPWM) {
//     // Apply raw PWM to motor.
//     motor->applyPWM(pwm);

//     // Run control ticks during settle period so velocity updates.
//     absolute_time_t settleStart = get_absolute_time();
//     while (absolute_time_diff_us(settleStart, get_absolute_time()) <
//            settleTimeMs * 1000) {
//       motor->controlTick();
//       sleep_ms(controlTickPeriodMs);
//     }

//     // Read steady-state velocity from motor.
//     float velocity = motor->getWheelVelocityMMPerSec();

//     sweepResults.push_back({pwm, velocity});
//     LOG_DEBUG("PWM=" + std::to_string(pwm) +
//               " | Velocity=" + std::to_string(velocity) + " mm/s.");
//   }

//   // Log all results in one go for Desmos.
//   std::string pwmList, velList;
//   for (size_t i = 0; i < sweepResults.size(); i++) {
//     if (sweepResults[i].measuredVelMMps != 0.0f) {
//       pwmList += std::to_string(sweepResults[i].appliedPWM);
//       velList += std::to_string(sweepResults[i].measuredVelMMps);
//       if (i != sweepResults.size() - 1) {
//         pwmList += ", ";
//         velList += ", ";
//       }
//     }
//   }
//   LOG_DEBUG("Velocity Values: x = [" + velList + "]");
//   LOG_DEBUG("PWM Values: p = [" + pwmList + "]");

//   motor->stopMotor();
//   return sweepResults;
// }
// /* Example Usage:
// LOG_DEBUG("Running right motor sweep...");
// // Sweep from 0.0 to 1.0 in steps of 0.05.
// runPWMSweep(&rightMotor, 0.0f, 1.0f, 0.05f, 10000, 25);
// sleep_ms(2000);
// LOG_DEBUG("Running right motor reverse sweep...");
// // Sweep from 0.0 to -1.0 in steps of -0.05.
// runPWMSweep(&rightMotor, 0.0f, -1.0f, -0.05f, 10000, 25);
// sleep_ms(2000);

// leftMotor.stopMotor();
// rightMotor.stopMotor();
// LOG_DEBUG("Motors stopped.");
// */

// /**
//  * Run step-based velocity test on both motors at given loop frequency.
//  * Useful for checking feedforward predictions against actual velocity
//  response.
//  * @param leftMotor Reference to left motor.
//  * @param rightMotor Reference to right motor.
//  * @param testVelocitiesMMps Vector of target velocities in mm/s to test.
//  * @param holdDurationSec Duration in seconds to hold each target velocity.
//  * @param loopPeriodMS Period in milliseconds to run control ticks and log
//  data.
//  *
//  * Note: Motor::controlTick() must call applyPWM() to actually drive motors.
//  */
// void runVelocityStepTest(Motor& leftMotor, Motor& rightMotor,
//                          const std::vector<float>& testVelocitiesMMps,
//                          float holdDurationSec, float loopPeriodMS) {
//   // Iterate through each test velocity.
//   for (float targetVel : testVelocitiesMMps) {
//     // Apply target velocity to both motors.
//     leftMotor.setDesiredVelocityMMPerSec(targetVel);
//     rightMotor.setDesiredVelocityMMPerSec(targetVel);

//     absolute_time_t startTime = get_absolute_time();
//     while (absolute_time_diff_us(startTime, get_absolute_time()) <
//            static_cast<int64_t>(holdDurationSec * 1e6f)) {
//       // Run control loop tick for both motors.
//       leftMotor.controlTick();
//       rightMotor.controlTick();

//       // Log current results.
//       LOG_DEBUG("TargetVel=" + std::to_string(targetVel) + " | LeftVel=" +
//                 std::to_string(leftMotor.getWheelVelocityMMPerSec()) +
//                 " | RightVel=" +
//                 std::to_string(rightMotor.getWheelVelocityMMPerSec()));

//       // Wait until next control cycle.
//       sleep_ms(loopPeriodMS);
//     }
//   }

//   // Stop motors at the end of the test.
//   leftMotor.stopMotor();
//   rightMotor.stopMotor();