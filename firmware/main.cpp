/**
 * main.cpp - Raspberry Pi Pico Micromouse Entry Point
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
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "app/api.h"
#include "app/bluetooth.h"
#include "app/commands.h"
#include "app/multicore.h"
#include "common/log.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "control/robot.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/motor.h"
#include "drivers/tof.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "motor_lab/motor_lab.h"
#include "control/line_follower.h"
#include "drivers/line_sensor.h"
#include "navigation/a_star.h"
#include "navigation/flood_fill.h"
#include "navigation/path_utils.h"
#include "pico/multicore.h"
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"

// ============================================================================
// Operating Mode Selection
// ============================================================================

enum class OperatingMode
{
    NORMAL,       // Dual-core maze solving
    MOTORLAB,     // Single-core motor characterization
    LINEFOLLOWING // Single-core line following with event queue
};

// Global mode (set during startup, read by runNormalMode/runMotorLabMode)
static OperatingMode g_operating_mode = OperatingMode::NORMAL;

// Global Battery Monitor (shared between cores in Normal mode)
static Battery* g_battery = nullptr;

// ============================================================================
// Dual Output printf (USB + Bluetooth UART) for MotorLab mode
// ============================================================================

// Custom driver to route stdout to Bluetooth UART
static void uart_out_chars(const char* buf, int length)
{
    for (int i = 0; i < length; i++)
    {
        uart_putc_raw(uart0, buf[i]);
    }
}

static stdio_driver_t bt_driver = {
    .out_chars = uart_out_chars,
    .out_flush = nullptr,
    .in_chars  = nullptr,
    .next      = nullptr,
};

// ============================================================================
// Startup Mode Selection
// ============================================================================

/**
 * Wait for mode selection input during startup.
 * Returns MOTORLAB if 'M' is pressed within timeout, otherwise NORMAL.
 */
OperatingMode selectOperatingMode(uint32_t timeout_ms)
{
    printf("\n");
    printf("==========================================\n");
    printf("  Jurababa Micromouse - Mode Selection   \n");
    printf("==========================================\n");
    printf("  Press 'M' within %lu seconds for MotorLab\n", timeout_ms / 1000);
    printf("  Press 'L' for Line Following\n");
    printf("  Otherwise, Normal mode starts...\n");
    printf("==========================================\n\n");

    uint32_t start_time        = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed           = 0;
    int      countdown_printed = -1;

    while (elapsed < timeout_ms)
    {
        // Check for input (USB serial)
        int c = getchar_timeout_us(100000); // Check every 100ms
        if (c != PICO_ERROR_TIMEOUT)
        {
            char ch = static_cast<char>(c);
            if (ch == 'M' || ch == 'm')
            {
                printf("\n*** MotorLab mode selected ***\n\n");
                return OperatingMode::MOTORLAB;
            }
            if (ch == 'L' || ch == 'l')
            {
                printf("\n*** Line Following mode selected ***\n\n");
                return OperatingMode::LINEFOLLOWING;
            }
        }

        elapsed = to_ms_since_boot(get_absolute_time()) - start_time;

        // Print countdown every second
        int seconds_left = static_cast<int>((timeout_ms - elapsed) / 1000);
        if (seconds_left != countdown_printed && seconds_left >= 0)
        {
            printf("  %d...\n", seconds_left);
            countdown_printed = seconds_left;
        }
    }

    printf("\n*** MotorLab mode starting ***\n\n");
    return OperatingMode::MOTORLAB;
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
                robot->moveDistance(HALF_CELL_DISTANCE_MM, 400.0f, 1000.0f);
                break;

            case CommandType::MOVE_FWD:
                robot->moveDistance(cmd.param * CELL_DISTANCE_MM, 400.0f, 1000.0f);
                break;

            case CommandType::CENTER_FROM_EDGE:
                robot->moveDistance(TO_CENTER_DISTANCE_MM, 400.0f, 1000.0f);
                break;

            case CommandType::TURN_LEFT:
                robot->turnInPlace(-90.0f * cmd.param, 360.0f, 720.0f);
                break;

            case CommandType::TURN_RIGHT:
                robot->turnInPlace(90.0f * cmd.param, 360.0f, 720.0f);
                break;

            case CommandType::TURN_ARBITRARY:
                robot->turnInPlace(static_cast<float>(cmd.param), 360.0f, 720.0f);
                break;

            case CommandType::ARC_TURN_LEFT_90:
                robot->smoothTurn(-90.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_RIGHT_90:
                robot->smoothTurn(90.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_LEFT_45:
                robot->smoothTurn(-45.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_RIGHT_45:
                robot->smoothTurn(45.0f, 45.0f);
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

    Drivetrain drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder, g_battery);
    Robot      robot(&drivetrain, &imu, &left_tof, &front_tof, &right_tof);

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

        SensorData sensor_data{};
        sensor_data.left_encoder  = left_encoder.ticks();
        sensor_data.right_encoder = right_encoder.ticks();
        sensor_data.tof_left_mm   = static_cast<int16_t>(left_tof.distance());
        sensor_data.tof_front_mm  = static_cast<int16_t>(front_tof.distance());
        sensor_data.tof_right_mm  = static_cast<int16_t>(right_tof.distance());
        sensor_data.imu_yaw       = imu.yaw();
        sensor_data.timestamp_ms  = to_ms_since_boot(now);
        SensorHub::publish(sensor_data);

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, CONTROL_PERIOD_MS);
    }
}

/**
 * Run Normal mode: dual-core maze solving.
 */
void runNormalMode(Battery& battery)
{
    Bluetooth bluetooth(uart0, 9600, 16, 17);
    bluetooth.init();

    bluetooth.write("=== Jurababa Normal Mode ===\r\n");

    Log::setBluetoothInterface(&bluetooth);
    Log::setBluetoothEnabled(true);
    LOG_INFO("Bluetooth logging enabled");

    LOG_DEBUG("Core0: Initializing multicore communication");
    SensorHub::init();
    multicore_launch_core1(core1_RobotController);

    multicore_fifo_pop_blocking();
    LOG_DEBUG("Core0: Core1 ready, starting maze solver");

    std::array<int, 2>              start_cell = {0, 0};
    std::vector<std::array<int, 2>> goal_cells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    Maze                            maze(MAZE_SIZE, MAZE_SIZE);
    Mouse                           mouse(start_cell, std::string("n"), goal_cells, &maze);
    API                             api(&mouse);

    LOG_INFO("Entering sensor monitoring loop...");

    uint32_t       last_battery_check_ms     = 0;
    const uint32_t BATTERY_CHECK_INTERVAL_MS = 5000;

    uint32_t       last_bt_print_ms     = 0;
    const uint32_t BT_PRINT_INTERVAL_MS = 1000;
    uint32_t       bt_message_count     = 0;

    while (true)
    {
        SensorData sensors;
        SensorHub::snapshot(sensors);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_battery_check_ms >= BATTERY_CHECK_INTERVAL_MS)
        {
            battery.update();
            last_battery_check_ms = now_ms;

            LOG_DEBUG("Battery: " << battery.voltage() << "V (raw ADC: " << battery.rawADC()
                                  << ")");

            if (battery.isLow(6.0f))
            {
                LOG_WARNING("Low battery! " << battery.voltage() << "V");
            }
        }

        if (now_ms - last_bt_print_ms >= BT_PRINT_INTERVAL_MS)
        {
            last_bt_print_ms = now_ms;
            bt_message_count++;

            LOG_INFO("[" << bt_message_count << "] "
                         << "Bat:" << battery.voltage() << "V "
                         << "L:" << sensors.tof_left_mm << " "
                         << "F:" << sensors.tof_front_mm << " "
                         << "R:" << sensors.tof_right_mm);
        }

        if (bluetooth.hasCommand())
        {
            Bluetooth::Command cmd = bluetooth.command();
            switch (cmd)
            {
                case Bluetooth::Command::START:
                    LOG_INFO("BT: Start command received");
                    break;
                case Bluetooth::Command::HALT:
                    LOG_INFO("BT: Halt command received");
                    CommandHub::send(CommandType::STOP);
                    break;
                case Bluetooth::Command::RESET:
                    LOG_INFO("BT: Reset command received");
                    break;
                case Bluetooth::Command::BATTERY:
                    battery.update();
                    bluetooth.write("Battery: " + std::to_string(battery.voltage()) + "V\r\n");
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
void runMotorLabMode(Battery& battery)
{
    // Set up UART for Bluetooth in MotorLab mode
    uart_init(uart0, 9600);
    gpio_set_function(16, GPIO_FUNC_UART);
    gpio_set_function(17, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    // Enable custom driver for Bluetooth output
    stdio_set_driver_enabled(&bt_driver, true);

    printf("\n\n");
    printf("===========================================\n");
    printf("  MOTORLAB MODE - Motor Characterization  \n");
    printf("===========================================\n");
    printf("Serial I/O: USB (115200) and UART0 (9600)\n");
    printf("Units: mm/s (Config.h compatible)\n\n");

    printf("Battery voltage: %.2f V\n", battery.voltage());

    Encoder left_encoder(pio0, 20, true);
    Encoder right_encoder(pio0, 8, false);
    Motor   left_motor(18, 19, true);
    Motor   right_motor(6, 7, false);

    MotorLab motorlab(&left_motor, &right_motor, &left_encoder, &right_encoder, &battery);
    motorlab.init();

    printf("\nHardware initialized. Ready for testing.\n");
    printf("Type '?' for command help.\n\n");

    const uint32_t  LOOP_PERIOD_MS = static_cast<uint32_t>(LOOP_INTERVAL_S * 1000.0f);
    absolute_time_t next_tick      = make_timeout_time_ms(LOOP_PERIOD_MS);

    uint32_t       last_battery_update_ms     = 0;
    const uint32_t BATTERY_UPDATE_INTERVAL_MS = 1000;

    while (true)
    {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        motorlab.updateEncoders(LOOP_INTERVAL_S);
        motorlab.processSerial();

        if (now_ms - last_battery_update_ms >= BATTERY_UPDATE_INTERVAL_MS)
        {
            battery.update();
            last_battery_update_ms = now_ms;
        }

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, LOOP_PERIOD_MS);
    }
}

// ============================================================================
// LINEFOLLOWING MODE - Line following with intersection event queue
// ============================================================================

/**
 * Run Line Following mode: single-core line following with event queue.
 *
 * The intersection_event_queue string defines actions at each intersection:
 *   L# = turn left 90°, R# = turn right 90°, F# = go forward
 * When the queue is exhausted, the robot continues forward.
 */
void runLineFollowingMode(Battery& battery)
{
    printf("\n");
    printf("==========================================\n");
    printf("  LINE FOLLOWING MODE                    \n");
    printf("==========================================\n\n");

    // Hardware init (no ToF sensors)
    Encoder left_encoder(pio0, PIN_ENCODER_L, true);
    Encoder right_encoder(pio0, PIN_ENCODER_R, false);
    Motor   left_motor(PIN_MOTOR_L_A, PIN_MOTOR_L_B, true);
    Motor   right_motor(PIN_MOTOR_R_A, PIN_MOTOR_R_B, false);
    IMU     imu(PIN_IMU_RX);

    LineSensor line_sensor(i2c0, PIN_LINE_SDA, PIN_LINE_SCL);
    line_sensor.init();

    Drivetrain   drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder, &battery);
    LineFollower line_follower(&drivetrain, &line_sensor, &imu, &battery);

    printf("Hardware initialized.\n");
    printf("Battery voltage: %.2f V\n", battery.voltage());

    // Intersection event queue: L=left, F=forward, R=right, separated by #
    std::string intersection_event_queue = "L#F#R#";

    printf("Event queue: %s\n\n", intersection_event_queue.c_str());

    // Parse the queue into individual tokens
    std::vector<char> event_queue;
    for (size_t i = 0; i < intersection_event_queue.size(); i++)
    {
        char ch = intersection_event_queue[i];
        if (ch == 'L' || ch == 'l' || ch == 'F' || ch == 'f' || ch == 'R' || ch == 'r')
        {
            event_queue.push_back(static_cast<char>(toupper(ch)));
        }
    }

    size_t queue_index = 0;

    printf("Parsed %zu events. Starting line follow...\n", event_queue.size());

    line_follower.startFollowing();

    // Control loop
    const int       CONTROL_PERIOD_MS = 10;
    absolute_time_t next_tick         = make_timeout_time_ms(CONTROL_PERIOD_MS);
    absolute_time_t last_tick         = get_absolute_time();

    while (true)
    {
        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;

        drivetrain.update(dt);
        line_follower.update(dt);

        // Check for intersection events
        if (line_follower.isIntersectionDetected() && line_follower.isMotionDone())
        {
            if (queue_index < event_queue.size())
            {
                char action = event_queue[queue_index];
                queue_index++;

                printf("Intersection! Action: %c (%zu/%zu)\n", action, queue_index,
                       event_queue.size());

                switch (action)
                {
                    case 'L':
                        line_follower.turnLeft90();
                        break;
                    case 'R':
                        line_follower.turnRight90();
                        break;
                    case 'F':
                    default:
                        // Continue forward — nothing to do
                        break;
                }
            }
            else
            {
                // Queue exhausted — keep going forward
                printf("Queue empty, continuing forward\n");
            }
        }

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, CONTROL_PERIOD_MS);
    }
}

// ============================================================================
// Startup LED Indicator
// ============================================================================

/**
 * Blink the onboard LED to indicate successful startup.
 */
void blinkStartupLED(int count, uint32_t on_ms, uint32_t off_ms)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < count; i++)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        if (i < count - 1)
        {
            sleep_ms(off_ms);
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main()
{
    // Initialize USB serial
    stdio_init_all();

    // Blink LED 3 times to indicate startup
    blinkStartupLED(3, 150, 150);

    sleep_ms(2000); // Wait for USB connection

    // Battery monitor setup (shared by both modes)
    Battery battery(26, 10000.0f, 5100.0f);
    battery.init();
    for (int i = 0; i < 10; i++)
    {
        battery.update();
        sleep_ms(10);
    }

    // Set global pointer for Core 1 access (Normal mode)
    g_battery = &battery;

    // Select operating mode (6 second window to press 'M')
    g_operating_mode = selectOperatingMode(6000);

    // Run selected mode
    if (g_operating_mode == OperatingMode::MOTORLAB)
    {
        runMotorLabMode(battery);
    }
    else if (g_operating_mode == OperatingMode::LINEFOLLOWING)
    {
        runLineFollowingMode(battery);
    }
    else
    {
        runNormalMode(battery);
    }

    return 0;
}
