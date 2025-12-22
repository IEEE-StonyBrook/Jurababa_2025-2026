#include <stdio.h>

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "../Include/Common/LogSystem.h"
#include "../Include/Navigation/AStarSolver.h"
#include "../Include/Platform/Pico/API.h"
#include "../Include/Platform/Pico/Config.h"
#include "../Include/Platform/Pico/MulticoreSensors.h"

#include "../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../Include/Platform/Pico/Robot/Encoder.h"
#include "../Include/Platform/Pico/Robot/IMU.h"
#include "../Include/Platform/Pico/Robot/Motor.h"
#include "../Include/Platform/Pico/Robot/Robot.h"
#include "../Include/Platform/Pico/Robot/Sensors.h"
#include "../Include/Platform/Pico/Robot/ToF.h"

#include "pico/stdlib.h"

void interpretLFRPath(API* apiPtr, std::string lfrPath);

// -------------------------
// Small angle helpers (Local to main)
// -------------------------
static float wrapDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

static float snapToNearest45Deg(float yawDeg)
{
    float snapped = 45.0f * std::round(yawDeg / 45.0f);
    snapped       = wrapDeg(snapped);
    if (snapped == -180.0f)
        snapped = 180.0f;
    return snapped;
}

// -------------------------
// Sensor publishing (Core1 -> Core0)
// -------------------------
void publishSensors(Encoder* leftEncoder, Encoder* rightEncoder, ToF* leftToF, ToF* frontToF,
                    ToF* rightToF, IMU* imu)
{
    MulticoreSensorData local{};
    local.left_encoder_count  = leftEncoder->getTickCount();
    local.right_encoder_count = rightEncoder->getTickCount();
    local.tof_left_mm         = static_cast<int16_t>(leftToF->getToFDistanceFromWallMM());
    local.tof_front_mm        = static_cast<int16_t>(frontToF->getToFDistanceFromWallMM());
    local.tof_right_mm        = static_cast<int16_t>(rightToF->getToFDistanceFromWallMM());
    local.imu_yaw             = imu->getIMUYawDegreesNeg180ToPos180();
    local.timestamp_ms        = to_ms_since_boot(get_absolute_time());

    MulticoreSensorHub::publish(local);
}

// -------------------------
// Core1 command handling
// - STOP interrupts immediately (mid-motion)
// - Other commands only start when robot is done
// - No blocking reads (prevents control-loop hiccups)
// -------------------------
void processCommands(Robot* robot, Sensors* sensors)
{
    CommandPacket cmd;
    bool          sawStop = false;

    // Drain all pending commands each tick
    while (CommandHub::receiveNonBlocking(cmd))
    {
        if (cmd.type == CommandType::STOP)
        {
            sawStop = true;
            continue; // Flush remaining commands behind STOP
        }

        // Never start a new motion while one is still running
        if (!robot->isMotionDone())
            continue;

        switch (cmd.type)
        {
            case CommandType::MOVE_FWD_HALF:
                robot->driveDistanceMM(HALF_CELL_DISTANCE_MM, FORWARD_TOP_SPEED);
                break;

            case CommandType::MOVE_FWD:
                robot->driveDistanceMM(cmd.param * CELL_DISTANCE_MM, FORWARD_TOP_SPEED);
                break;

            case CommandType::CENTER_FROM_EDGE:
                robot->driveDistanceMM(TO_CENTER_DISTANCE_MM, FORWARD_TOP_SPEED);
                break;

            case CommandType::TURN_LEFT:
            {
                float currYaw = sensors->getYaw();
                float desired = wrapDeg(currYaw - static_cast<float>(cmd.param));
                robot->turnToYawDeg(snapToNearest45Deg(desired));
            }
            break;

            case CommandType::TURN_RIGHT:
            {
                float currYaw = sensors->getYaw();
                float desired = wrapDeg(currYaw + static_cast<float>(cmd.param));
                robot->turnToYawDeg(snapToNearest45Deg(desired));
            }
            break;

            case CommandType::TURN_ARBITRARY:
            {
                float currYaw = sensors->getYaw();
                float desired = wrapDeg(currYaw + static_cast<float>(cmd.param)); // signed ok
                robot->turnToYawDeg(snapToNearest45Deg(desired));
            }
            break;

            // Core0 may send this, but Core1 doesn't need to act on it
            case CommandType::SNAPSHOT:
            case CommandType::NONE:
            default:
                break;
        }
    }

    if (sawStop)
    {
        robot->stop();
    }
}

// Publisher for Core1: All robot specific logic
static void core1_Publisher()
{
    // Sensors
    Encoder leftEncoder(pio0, 20, true);
    Encoder rightEncoder(pio0, 8, false);
    ToF     leftToF(11, 'L');
    ToF     frontToF(12, 'F');
    ToF     rightToF(13, 'R');
    IMU     imu(5);
    imu.resetIMUYawToZero();

    // Motors
    Motor leftMotor(18, 19, true);
    Motor rightMotor(6, 7, false);

    // Updated stack (Robot owns motion)
    Drivetrain drivetrain(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder);
    Sensors    sensors(&imu, &leftToF, &frontToF, &rightToF);
    Robot      robot(&drivetrain, &sensors);

    LOG_DEBUG("Initialization complete.");
    robot.reset();

    multicore_fifo_push_blocking(1); // Signal Core0 that Core1 is ready

    const float dt = static_cast<float>(CORE_SLEEP_MS) / 1000.0f;

    while (true)
    {
        robot.update(dt);

        // Mid-motion STOP interrupt + sequential command start
        processCommands(&robot, &sensors);

        publishSensors(&leftEncoder, &rightEncoder, &leftToF, &frontToF, &rightToF, &imu);

        sleep_ms(CORE_SLEEP_MS);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(3000);

    MulticoreSensorHub::init();
    multicore_launch_core1(core1_Publisher);

    // Wait until Core1 signals it finished initializing its sensors
    multicore_fifo_pop_blocking();

    // Maze / planning objects
    std::array<int, 2>              startCell = {0, 0};
    std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    MazeGraph                       maze(16, 16);
    InternalMouse                   mouse(startCell, std::string("n"), goalCells, &maze);
    API                             api(&mouse);

    api.goToCenterFromEdge();
    api.executeSequence("F#F#F#F#L#F#");

    // Core0 loop: read sensor snapshots, update maze state / planning.
    while (true)
    {
        MulticoreSensorData sensors;
        MulticoreSensorHub::snapshot(sensors);

        if (sensors.tof_left_exist)
            mouse.setWallExistsLFR('L');
        if (sensors.tof_front_exist)
            mouse.setWallExistsLFR('F');
        if (sensors.tof_right_exist)
            mouse.setWallExistsLFR('R');

        sleep_ms(CORE_SLEEP_MS);
    }

    return 0;
}

void interpretLFRPath(API* apiPtr, std::string lfrPath)
{
    std::stringstream ss(lfrPath);
    std::string       token;

    std::vector<std::string> tokens;
    while (std::getline(ss, token, '#'))
    {
        if (!token.empty())
        {
            tokens.push_back(token);
        }
    }

    for (std::string t : tokens)
    {
        if (t == "R")
        {
            apiPtr->turnRight90();
        }
        else if (t == "L")
        {
            apiPtr->turnLeft90();
        }
        else if (t == "F")
        {
            apiPtr->moveForward();
        }
        else if (t == "R45")
        {
            apiPtr->turnRight45();
        }
        else if (t == "L45")
        {
            apiPtr->turnLeft45();
        }
        else if (t == "FH")
        {
            apiPtr->moveForwardHalf();
        }
        else
        {
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
//  * Note: Comment Motor::controlTick() applyDuty() call to run open-loop tests.
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
//     motor->applyDuty(pwm);

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
//  * Note: Motor::controlTick() must call applyDuty() to actually drive motors.
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