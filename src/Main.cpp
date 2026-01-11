/**
 * Main.cpp - Raspberry Pi Pico Micromouse Entry Point
 *
 * Architecture:
 *   Normal Mode: Dual-core maze solving
 *     - Core 0: High-level maze solving and path planning
 *     - Core 1: Real-time robot control and sensor management
 *   MotorLab Mode: Single-core motor characterization
 *     - Motor calibration CLI for tuning feedforward/PID constants
 *
 * Mode Selection:
 *   Press 'M' within 3 seconds of startup to enter MotorLab mode.
 *   Otherwise, Normal mode starts automatically.
 */

#include <array>
#include <stdio.h>
#include <stdarg.h>
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
#include "Platform/Pico/MotorLab/MotorLab.h"
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
// Operating Mode Selection
// ============================================================================

enum class OperatingMode {
    NORMAL,     // Dual-core maze solving
    MOTORLAB    // Single-core motor characterization
};

// Global mode (set during startup, read by runNormalMode/runMotorLabMode)
static OperatingMode g_operating_mode = OperatingMode::NORMAL;

// Global Battery Monitor (shared between cores in Normal mode)
static BatteryMonitor* g_battery_monitor = nullptr;

// ============================================================================
// Dual Output printf (USB + Bluetooth UART) for MotorLab mode
// ============================================================================

static void uart_write_str(const char* str) {
    while (*str) {
        uart_putc_raw(uart0, *str++);
    }
}

// Custom printf that outputs to both USB and Bluetooth UART
// Only used in MotorLab mode for CLI output
static int motorlab_printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Send to USB
    fputs(buffer, stdout);
    fflush(stdout);

    // Send to UART (Bluetooth)
    uart_write_str(buffer);

    return ret;
}

// ============================================================================
// Startup Mode Selection
// ============================================================================

/**
 * Wait for mode selection input during startup.
 * Returns MOTORLAB if 'M' is pressed within timeout, otherwise NORMAL.
 */
OperatingMode selectOperatingMode(uint32_t timeout_ms) {
    printf("\n");
    printf("==========================================\n");
    printf("  Jurababa Micromouse - Mode Selection   \n");
    printf("==========================================\n");
    printf("  Press 'M' within %lu seconds for MotorLab\n", timeout_ms / 1000);
    printf("  Otherwise, Normal mode starts...\n");
    printf("==========================================\n\n");

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = 0;
    int countdown_printed = -1;

    while (elapsed < timeout_ms) {
        // Check for input (USB serial)
        int c = getchar_timeout_us(100000);  // Check every 100ms
        if (c != PICO_ERROR_TIMEOUT) {
            char ch = static_cast<char>(c);
            if (ch == 'M' || ch == 'm') {
                printf("\n*** MotorLab mode selected ***\n\n");
                return OperatingMode::MOTORLAB;
            }
        }

        elapsed = to_ms_since_boot(get_absolute_time()) - start_time;

        // Print countdown every second
        int seconds_left = static_cast<int>((timeout_ms - elapsed) / 1000);
        if (seconds_left != countdown_printed && seconds_left >= 0) {
            printf("  %d...\n", seconds_left);
            countdown_printed = seconds_left;
        }
    }

    printf("\n*** Normal mode starting ***\n\n");
    return OperatingMode::NORMAL;
}

// ============================================================================
// NORMAL MODE - Maze-solving operation with dual-core architecture
// ============================================================================

/**
 * Processes motion commands from Core 0.
 * Handles STOP immediately, queues other commands when robot is ready.
 */
void processCommands(Robot* robot)
{
    CommandPacket cmd;
    bool          saw_stop = false;

    while (CommandHub::receiveNonBlocking(cmd))
    {
        if (cmd.type == CommandType::STOP)
        {
            saw_stop = true;
            continue;
        }

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
 */
void core1_RobotController()
{
    Encoder left_encoder(pio0, 20, true);
    Encoder right_encoder(pio0, 8, false);
    ToF     left_tof(11, 'L');
    ToF     front_tof(12, 'F');
    ToF     right_tof(13, 'R');
    IMU     imu(5);
    Motor   left_motor(18, 19, true);
    Motor   right_motor(6, 7, false);

    Drivetrain drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder, g_battery_monitor);
    Sensors    sensors(&imu, &left_tof, &front_tof, &right_tof);
    Robot      robot(&drivetrain, &sensors);

    LOG_DEBUG("Core1: Hardware initialized");
    robot.reset();

    multicore_fifo_push_blocking(1);

    const int       CONTROL_PERIOD_MS = 10;
    absolute_time_t next_tick         = make_timeout_time_ms(CONTROL_PERIOD_MS);
    absolute_time_t last_tick         = get_absolute_time();

    while (true)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;

        robot.update(dt);
        processCommands(&robot);

        MulticoreSensorData sensor_data{};
        sensor_data.left_encoder_count  = left_encoder.getTickCount();
        sensor_data.right_encoder_count = right_encoder.getTickCount();
        sensor_data.tof_left_mm         = static_cast<int16_t>(left_tof.getToFDistanceFromWallMM());
        sensor_data.tof_front_mm        = static_cast<int16_t>(front_tof.getToFDistanceFromWallMM());
        sensor_data.tof_right_mm        = static_cast<int16_t>(right_tof.getToFDistanceFromWallMM());
        sensor_data.imu_yaw             = imu.getIMUYawDegreesNeg180ToPos180();
        sensor_data.timestamp_ms        = to_ms_since_boot(now);
        MulticoreSensorHub::publish(sensor_data);

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, CONTROL_PERIOD_MS);
    }
}

/**
 * Run Normal mode: dual-core maze solving.
 */
void runNormalMode(BatteryMonitor& battery)
{
    BluetoothInterface bluetooth(uart0, 9600, 16, 17);
    bluetooth.init();

    bluetooth.write("=== Jurababa Normal Mode ===\r\n");

    LogSystem::setBluetoothInterface(&bluetooth);
    LogSystem::setBluetoothEnabled(true);
    LOG_INFO("Bluetooth logging enabled");

    LOG_DEBUG("Core0: Initializing multicore communication");
    MulticoreSensorHub::init();
    multicore_launch_core1(core1_RobotController);

    multicore_fifo_pop_blocking();
    LOG_DEBUG("Core0: Core1 ready, starting maze solver");

    std::array<int, 2>              start_cell = {0, 0};
    std::vector<std::array<int, 2>> goal_cells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    MazeGraph                       maze(MAZE_SIZE, MAZE_SIZE);
    InternalMouse                   mouse(start_cell, std::string("n"), goal_cells, &maze);
    API                             api(&mouse);

    LOG_INFO("Entering sensor monitoring loop...");

    uint32_t last_battery_check_ms = 0;
    const uint32_t BATTERY_CHECK_INTERVAL_MS = 5000;

    uint32_t last_bt_print_ms = 0;
    const uint32_t BT_PRINT_INTERVAL_MS = 1000;
    uint32_t bt_message_count = 0;

    while (true)
    {
        MulticoreSensorData sensors;
        MulticoreSensorHub::snapshot(sensors);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_battery_check_ms >= BATTERY_CHECK_INTERVAL_MS)
        {
            battery.update();
            last_battery_check_ms = now_ms;

            LOG_DEBUG("Battery: " << battery.getVoltage() << "V (raw ADC: " << battery.getRawADC() << ")");

            if (battery.isLowBattery(6.0f))
            {
                LOG_WARNING("Low battery! " << battery.getVoltage() << "V");
            }
        }

        if (now_ms - last_bt_print_ms >= BT_PRINT_INTERVAL_MS)
        {
            last_bt_print_ms = now_ms;
            bt_message_count++;

            LOG_INFO("[" << bt_message_count << "] "
                     << "Bat:" << battery.getVoltage() << "V "
                     << "L:" << sensors.tof_left_mm << " "
                     << "F:" << sensors.tof_front_mm << " "
                     << "R:" << sensors.tof_right_mm);
        }

        if (bluetooth.hasCommand())
        {
            BluetoothInterface::Command cmd = bluetooth.getCommand();
            switch (cmd)
            {
                case BluetoothInterface::Command::START:
                    LOG_INFO("BT: Start command received");
                    break;
                case BluetoothInterface::Command::HALT:
                    LOG_INFO("BT: Halt command received");
                    CommandHub::send(CommandType::STOP);
                    break;
                case BluetoothInterface::Command::RESET:
                    LOG_INFO("BT: Reset command received");
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
}

// ============================================================================
// MOTORLAB MODE - Motor characterization and tuning interface
// ============================================================================

/**
 * Run MotorLab mode: single-core motor characterization CLI.
 */
void runMotorLabMode(BatteryMonitor& battery)
{
    // Set up UART for Bluetooth in MotorLab mode
    uart_init(uart0, 9600);
    gpio_set_function(16, GPIO_FUNC_UART);
    gpio_set_function(17, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    motorlab_printf("\n\n");
    motorlab_printf("===========================================\n");
    motorlab_printf("  MOTORLAB MODE - Motor Characterization  \n");
    motorlab_printf("===========================================\n");
    motorlab_printf("Serial I/O: USB (115200) and UART0 (9600)\n");
    motorlab_printf("Units: mm/s (Config.h compatible)\n\n");

    motorlab_printf("Battery voltage: %.2f V\n", battery.getVoltage());

    Encoder left_encoder(pio0, 20, true);
    Encoder right_encoder(pio0, 8, false);
    Motor   left_motor(18, 19, true);
    Motor   right_motor(6, 7, false);

    MotorLab motorlab(&left_motor, &right_motor,
                      &left_encoder, &right_encoder,
                      &battery);
    motorlab.init();

    motorlab_printf("\nHardware initialized. Ready for testing.\n");
    motorlab_printf("Type '?' for command help.\n\n");

    const uint32_t LOOP_PERIOD_MS = static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f);
    absolute_time_t next_tick = make_timeout_time_ms(LOOP_PERIOD_MS);

    uint32_t last_battery_update_ms = 0;
    const uint32_t BATTERY_UPDATE_INTERVAL_MS = 1000;

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        motorlab.updateEncoders(LOOP_INTERVAL_S);
        motorlab.processSerial();

        if (now_ms - last_battery_update_ms >= BATTERY_UPDATE_INTERVAL_MS) {
            battery.update();
            last_battery_update_ms = now_ms;
        }

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, LOOP_PERIOD_MS);
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main()
{
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB connection

    // Battery monitor setup (shared by both modes)
    BatteryMonitor battery(26, 10000.0f, 5100.0f);
    battery.init();
    for (int i = 0; i < 10; i++) {
        battery.update();
        sleep_ms(10);
    }

    // Set global pointer for Core 1 access (Normal mode)
    g_battery_monitor = &battery;

    // Select operating mode (3 second window to press 'M')
    g_operating_mode = selectOperatingMode(3000);

    // Run selected mode
    if (g_operating_mode == OperatingMode::MOTORLAB) {
        runMotorLabMode(battery);
    } else {
        runNormalMode(battery);
    }

    return 0;
}
