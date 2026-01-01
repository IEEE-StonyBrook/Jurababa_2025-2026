/**
 * Main.cpp - Raspberry Pi Pico Micromouse Entry Point
 *
 * Architecture:
 *   Core 0: High-level maze solving and path planning
 *   Core 1: Real-time robot control and sensor management
 *
 * Communication:
 *   - Core 0 → Core 1: Commands via CommandHub (non-blocking FIFO)
 *   - Core 1 → Core 0: Sensor data via MulticoreSensorHub
 */

#include <stdio.h>
#include <array>
#include <string>
#include <vector>

#include "Common/LogSystem.h"
#include "Maze/InternalMouse.h"
#include "Maze/MazeGraph.h"
#include "Navigation/AStarSolver.h"
#include "Navigation/FrontierBasedSearchSolver.h"
#include "Navigation/PathUtils.h"
#include "Platform/Pico/API.h"
#include "Platform/Pico/CommandHub.h"
#include "Platform/Pico/Config.h"
#include "Platform/Pico/MulticoreSensors.h"
#include "Platform/Pico/Robot/Drivetrain.h"
#include "Platform/Pico/Robot/Encoder.h"
#include "Platform/Pico/Robot/IMU.h"
#include "Platform/Pico/Robot/Motor.h"
#include "Platform/Pico/Robot/Robot.h"
#include "Platform/Pico/Robot/Sensors.h"
#include "Platform/Pico/Robot/ToF.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// ============================================================================
// CORE 1: Real-Time Robot Control
// ============================================================================

/**
 * Processes motion commands from Core 0.
 * Handles STOP immediately, queues other commands when robot is ready.
 */
void processCommands(Robot* robot)
{
    CommandPacket cmd;
    bool sawStop = false;

    // Drain all pending commands each tick
    while (CommandHub::receiveNonBlocking(cmd))
    {
        if (cmd.type == CommandType::STOP)
        {
            sawStop = true;
            continue; // Flush remaining commands behind STOP
        }

        // Never start new motion while one is running
        if (!robot->isMotionDone())
            continue;

        switch (cmd.type)
        {
            case CommandType::MOVE_FWD_HALF:
                robot->driveDistanceMM(HALF_CELL_DISTANCE_MM, 400.0f);
                break;

            case CommandType::MOVE_FWD:
                robot->driveDistanceMM(cmd.param * CELL_DISTANCE_MM, 400.0f);
                break;

            case CommandType::CENTER_FROM_EDGE:
                robot->driveDistanceMM(TO_CENTER_DISTANCE_MM, 400.0f);
                break;

            case CommandType::TURN_LEFT:
                robot->turn45Degrees("left", cmd.param);
                break;

            case CommandType::TURN_RIGHT:
                robot->turn45Degrees("right", cmd.param);
                break;

            case CommandType::TURN_ARBITRARY:
            {
                float stepsOf45 = static_cast<float>(cmd.param) / 45.0f;
                if (stepsOf45 > 0)
                    robot->turn45Degrees("right", static_cast<int>(stepsOf45));
                else
                    robot->turn45Degrees("left", static_cast<int>(-stepsOf45));
                break;
            }

            case CommandType::ARC_TURN_LEFT_90:
                robot->arcTurn90Degrees("left");
                break;

            case CommandType::ARC_TURN_RIGHT_90:
                robot->arcTurn90Degrees("right");
                break;

            case CommandType::ARC_TURN_LEFT_45:
                robot->arcTurn45Degrees("left");
                break;

            case CommandType::ARC_TURN_RIGHT_45:
                robot->arcTurn45Degrees("right");
                break;

            case CommandType::SNAPSHOT:
            case CommandType::NONE:
            default:
                break;
        }
    }

    if (sawStop)
        robot->stop();
}

/**
 * Core 1 entry point: Runs real-time control loop at 100Hz.
 * Manages sensors, motors, and executes motion commands from Core 0.
 */
void core1_RobotController()
{
    // Initialize hardware
    Encoder leftEncoder(pio0, 20, true);
    Encoder rightEncoder(pio0, 8, false);
    ToF     leftToF(11, 'L');
    ToF     frontToF(12, 'F');
    ToF     rightToF(13, 'R');
    IMU     imu(5);
    Motor   leftMotor(18, 19, true);
    Motor   rightMotor(6, 7, false);

    // Initialize control stack
    Drivetrain drivetrain(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder);
    Sensors    sensors(&imu, &leftToF, &frontToF, &rightToF);
    Robot      robot(&drivetrain, &sensors);

    LOG_DEBUG("Core1: Hardware initialized");
    robot.reset();

    // Signal Core 0 that initialization is complete
    multicore_fifo_push_blocking(1);

    // Real-time control loop (100Hz = 10ms period)
    const int CONTROL_PERIOD_MS = 10;
    absolute_time_t nextTick = make_timeout_time_ms(CONTROL_PERIOD_MS);
    absolute_time_t lastTick = get_absolute_time();

    while (true)
    {
        // Calculate delta time for control updates
        absolute_time_t now = get_absolute_time();
        float dt = absolute_time_diff_us(lastTick, now) * 1e-6f;
        lastTick = now;

        // Update robot control (PID, motion profiling, etc.)
        robot.update(dt);

        // Process any pending commands from Core 0
        processCommands(&robot);

        // Publish sensor data to Core 0
        MulticoreSensorData sensorData{};
        sensorData.left_encoder_count = leftEncoder.getTickCount();
        sensorData.right_encoder_count = rightEncoder.getTickCount();
        sensorData.tof_left_mm = static_cast<int16_t>(leftToF.getToFDistanceFromWallMM());
        sensorData.tof_front_mm = static_cast<int16_t>(frontToF.getToFDistanceFromWallMM());
        sensorData.tof_right_mm = static_cast<int16_t>(rightToF.getToFDistanceFromWallMM());
        sensorData.imu_yaw = imu.getIMUYawDegreesNeg180ToPos180();
        sensorData.timestamp_ms = to_ms_since_boot(now);
        MulticoreSensorHub::publish(sensorData);

        // Sleep until next control tick
        sleep_until(nextTick);
        nextTick = delayed_by_ms(nextTick, CONTROL_PERIOD_MS);
    }
}

// ============================================================================
// CORE 0: High-Level Maze Solving
// ============================================================================

/**
 * Main entry point: Runs maze solving algorithms on Core 0.
 * Sends motion commands to Core 1 via CommandHub.
 */
int main()
{
    // Initialize USB serial for debugging
    stdio_init_all();
    sleep_ms(3000);

    LOG_DEBUG("Core0: Initializing multicore communication");
    MulticoreSensorHub::init();
    multicore_launch_core1(core1_RobotController);

    // Wait for Core 1 to complete hardware initialization
    multicore_fifo_pop_blocking();
    LOG_DEBUG("Core0: Core1 ready, starting maze solver");

    // Initialize maze solving components
    std::array<int, 2> startCell = {0, 0};
    std::vector<std::array<int, 2>> goalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    MazeGraph maze(16, 16);
    InternalMouse mouse(startCell, std::string("n"), goalCells, &maze);
    API api(&mouse);

    // ========================================================================
    // Algorithm Execution - Modify this section for different solving modes
    // ========================================================================

    // EXPLORATION MODE: Use frontier-based search to map unknown maze
    // Uncomment to enable:
    // FrontierBasedSearchSolver frontierSolver(&mouse);
    // bool explorationComplete = traversePathIteratively(&api, &mouse, goalCells, false, false, false);
    // if (explorationComplete) {
    //     LOG_INFO("Maze exploration complete!");
    // }

    // SPEED RUN MODE: Use A* with known maze layout
    // Uncomment to enable:
    // setAllExplored(&mouse);  // Mark entire maze as explored
    // bool speedRunComplete = traversePathIteratively(&api, &mouse, goalCells, true, true, false);
    // if (speedRunComplete) {
    //     LOG_INFO("Speed run complete!");
    // }

    // TEST MODE: Execute a simple test sequence
    LOG_INFO("Test mode: Executing square pattern");
    api.executeSequence("F#R#F#R#F#R#F#R");

    // ========================================================================
    // Sensor monitoring loop - update maze state from real-time sensor data
    // ========================================================================
    while (true)
    {
        MulticoreSensorData sensors;
        MulticoreSensorHub::snapshot(sensors);

        // Update maze walls based on ToF sensor readings
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
