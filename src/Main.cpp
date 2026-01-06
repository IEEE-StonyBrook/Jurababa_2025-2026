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

#include <array>
#include <stdio.h>
#include <string>
#include <vector>

#include "Common/LogSystem.h"
#include "Maze/InternalMouse.h"
#include "Maze/MazeGraph.h"
#include "Navigation/AStarSolver.h"
#include "Navigation/FloodFillSolver.h"
#include "Navigation/PathUtils.h"
#include "Platform/Pico/API.h"
#include "Platform/Pico/BluetoothInterface.h"
#include "Platform/Pico/CommandHub.h"
#include "Platform/Pico/Config.h"
#include "Platform/Pico/MulticoreSensors.h"
#include "Platform/Pico/Robot/BatteryMonitor.h"
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
// Global Battery Monitor (shared between cores)
// ============================================================================
// Core 0 owns and updates this; Core 1 reads via pointer for voltage scaling.
// Access is safe because reads are atomic for aligned floats on ARM Cortex-M0+.
static BatteryMonitor* g_battery_monitor = nullptr;

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
    bool          saw_stop = false;

    // Drain all pending commands each tick
    while (CommandHub::receiveNonBlocking(cmd))
    {
        if (cmd.type == CommandType::STOP)
        {
            saw_stop = true;
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
                float steps_of_45 = static_cast<float>(cmd.param) / 45.0f;
                if (steps_of_45 > 0)
                    robot->turn45Degrees("right", static_cast<int>(steps_of_45));
                else
                    robot->turn45Degrees("left", static_cast<int>(-steps_of_45));
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

    if (saw_stop)
        robot->stop();
}

/**
 * Core 1 entry point: Runs real-time control loop at 100Hz.
 * Manages sensors, motors, and executes motion commands from Core 0.
 */
void core1_RobotController()
{
    // Initialize hardware
    Encoder left_encoder(pio0, 20, true);
    Encoder right_encoder(pio0, 8, false);
    ToF     left_tof(11, 'L');
    ToF     front_tof(12, 'F');
    ToF     right_tof(13, 'R');
    IMU     imu(5);
    Motor   left_motor(18, 19, true);
    Motor   right_motor(6, 7, false);

    // Initialize control stack (with battery monitor for voltage-based control)
    Drivetrain drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder, g_battery_monitor);
    Sensors    sensors(&imu, &left_tof, &front_tof, &right_tof);
    Robot      robot(&drivetrain, &sensors);

    LOG_DEBUG("Core1: Hardware initialized");
    robot.reset();

    // Signal Core 0 that initialization is complete
    multicore_fifo_push_blocking(1);

    // Real-time control loop (100Hz = 10ms period)
    const int       CONTROL_PERIOD_MS = 10;
    absolute_time_t next_tick         = make_timeout_time_ms(CONTROL_PERIOD_MS);
    absolute_time_t last_tick         = get_absolute_time();

    while (true)
    {
        // Calculate delta time for control updates
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;

        // Update robot control (PID, motion profiling, etc.)
        robot.update(dt);

        // Process any pending commands from Core 0
        processCommands(&robot);

        // Publish sensor data to Core 0
        MulticoreSensorData sensor_data{};
        sensor_data.left_encoder_count  = left_encoder.getTickCount();
        sensor_data.right_encoder_count = right_encoder.getTickCount();
        sensor_data.tof_left_mm         = static_cast<int16_t>(left_tof.getToFDistanceFromWallMM());
        sensor_data.tof_front_mm        = static_cast<int16_t>(front_tof.getToFDistanceFromWallMM());
        sensor_data.tof_right_mm        = static_cast<int16_t>(right_tof.getToFDistanceFromWallMM());
        sensor_data.imu_yaw             = imu.getIMUYawDegreesNeg180ToPos180();
        sensor_data.timestamp_ms        = to_ms_since_boot(now);
        MulticoreSensorHub::publish(sensor_data);

        // Sleep until next control tick
        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, CONTROL_PERIOD_MS);
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

    // ========================================================================
    // Battery Monitor Setup (ADC0 on GP26)
    // ========================================================================
    BatteryMonitor battery(26, 10000.0f, 5100.0f);  // R1=10k, R2=5.1k
    battery.init();

    // Take initial battery readings to fill the moving average buffer
    for (int i = 0; i < 10; i++)
    {
        battery.update();
        sleep_ms(10);
    }
    LOG_INFO("Battery voltage: " << battery.getVoltage() << "V");

    if (battery.isLowBattery(6.0f))
    {
        LOG_WARNING("Low battery detected! Voltage: " << battery.getVoltage() << "V");
    }

    // Set global pointer for Core 1 access (before launching Core 1)
    g_battery_monitor = &battery;

    // ========================================================================
    // Bluetooth Setup (UART0, GP16=TX, GP17=RX)
    // ========================================================================
    BluetoothInterface bluetooth(uart0, 9600, 16, 17);
    bluetooth.init();

    // Direct Bluetooth test - bypasses LogSystem
    bluetooth.write("=== HM-10 Bluetooth Test ===\r\n");
    bluetooth.write("If you see this, Bluetooth TX is working!\r\n");

    LogSystem::setBluetoothInterface(&bluetooth);
    LogSystem::setBluetoothEnabled(true);
    LOG_INFO("Bluetooth logging enabled");

    LOG_DEBUG("Core0: Initializing multicore communication");
    MulticoreSensorHub::init();
    multicore_launch_core1(core1_RobotController);

    // Wait for Core 1 to complete hardware initialization
    multicore_fifo_pop_blocking();
    LOG_DEBUG("Core0: Core1 ready, starting maze solver");

    // Initialize maze solving components
    std::array<int, 2>              start_cell = {0, 0};
    std::vector<std::array<int, 2>> goal_cells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    MazeGraph                       maze(MAZE_SIZE, MAZE_SIZE);
    InternalMouse                   mouse(start_cell, std::string("n"), goal_cells, &maze);
    API                             api(&mouse);

    // ========================================================================
    // Algorithm Execution - Modify this section for different solving modes
    // ========================================================================

    // EXPLORATION MODE: Use flood-fill search to map unknown maze
    // Uncomment to enable:
    // FloodFillSolver::explore(mouse, api, false);
    // bool exploration_complete = traversePathIteratively(&api, &mouse, goal_cells, false, false,
    // false); if (exploration_complete) {
    //     LOG_INFO("Maze exploration complete!");
    // }

    // SPEED RUN MODE: Use A* with known maze layout
    // Uncomment to enable:
    // setAllExplored(&mouse);  // Mark entire maze as explored
    // bool speed_run_complete = traversePathIteratively(&api, &mouse, goal_cells, true, true, false);
    // if (speed_run_complete) {
    //     LOG_INFO("Speed run complete!");
    // }

    // TEST MODE: Execute a simple test sequence
    // LOG_INFO("Test mode: Executing square pattern");
    // api.executeSequence("F#R#F#R#F#R#F#R");
    LOG_INFO("Entering sensor monitoring loop...");

    // ========================================================================
    // Sensor monitoring loop - update maze state from real-time sensor data
    // ========================================================================
    uint32_t last_battery_check_ms = 0;
    const uint32_t BATTERY_CHECK_INTERVAL_MS = 5000;  // Check every 5 seconds

    uint32_t last_bt_print_ms = 0;
    const uint32_t BT_PRINT_INTERVAL_MS = 1000;  // Print status every 1 second
    uint32_t bt_message_count = 0;

    while (true)
    {
        MulticoreSensorData sensors;
        MulticoreSensorHub::snapshot(sensors);

        // Update maze walls based on ToF sensor readings
        // if (sensors.tof_left_exist)
        //     mouse.setWallExistsLFR('L');
        // if (sensors.tof_front_exist)
        //     mouse.setWallExistsLFR('F');
        // if (sensors.tof_right_exist)
        //     mouse.setWallExistsLFR('R');

        // Periodic battery voltage check (non-blocking)
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_battery_check_ms >= BATTERY_CHECK_INTERVAL_MS)
        {
            battery.update();
            last_battery_check_ms = now_ms;

            // Log battery status periodically
            LOG_DEBUG("Battery: " << battery.getVoltage() << "V (raw ADC: " << battery.getRawADC() << ")");

            if (battery.isLowBattery(6.0f))
            {
                LOG_WARNING("Low battery! " << battery.getVoltage() << "V");
            }
        }

        // ====================================================================
        // Continuous Status Print (for testing serial + Bluetooth connection)
        // ====================================================================
        if (now_ms - last_bt_print_ms >= BT_PRINT_INTERVAL_MS)
        {
            last_bt_print_ms = now_ms;
            bt_message_count++;

            // Print status message with counter, battery, and sensor data
            // LOG_INFO outputs to both USB serial and Bluetooth
            LOG_INFO("[" << bt_message_count << "] "
                     << "Bat:" << battery.getVoltage() << "V "
                     << "L:" << sensors.tof_left_mm << " "
                     << "F:" << sensors.tof_front_mm << " "
                     << "R:" << sensors.tof_right_mm);
        }

        // ====================================================================
        // Bluetooth Command Handling
        // ====================================================================
        if (bluetooth.hasCommand())
        {
            BluetoothInterface::Command cmd = bluetooth.getCommand();
            switch (cmd)
            {
                case BluetoothInterface::Command::START:
                    LOG_INFO("BT: Start command received");
                    // Add start logic here
                    break;
                case BluetoothInterface::Command::HALT:
                    LOG_INFO("BT: Halt command received");
                    CommandHub::send(CommandType::STOP);
                    break;
                case BluetoothInterface::Command::RESET:
                    LOG_INFO("BT: Reset command received");
                    // Add reset logic here
                    break;
                case BluetoothInterface::Command::BATTERY:
                    battery.update();
                    bluetooth.write("Battery: " + std::to_string(battery.getVoltage()) + "V\r\n");
                    break;
                default:
                    LOG_DEBUG("BT: Unknown command");
                    break;
            }
        }

        sleep_ms(CORE_SLEEP_MS);
    }

    return 0;
}
