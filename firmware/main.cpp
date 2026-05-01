/**
 * main.cpp — Raspberry Pi Pico Micromouse entry point.
 *
 * Two top-level operating modes:
 *
 *   1. DriverLab (press 'M' within 3 s of boot)
 *        Single-core motor characterization CLI. Core 0 owns motors,
 *        encoders, IMU, and either ToFs or the line sensor (mutually
 *        exclusive on I2C0).
 *
 *   2. Cli (default)
 *        UKMARS mazerunner-core-style command loop. Same dispatch table
 *        for both ToF and LineSensor sub-modes:
 *          - TOF        : Core 1 launches and runs `Robot` at 500 Hz;
 *                         Core 0 runs the maze brain (FloodFill, A*).
 *          - LINE_SENSOR: Core 1 does not launch; Core 0 owns motors via
 *                         `LineFollower`.
 *
 * At boot:
 *   Press 'M' within 3 s → DriverLab.
 *   Else → prompt for sensor mode (T/L), then run Cli.
 */

#include <array>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "pico/multicore.h"
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/cli.h"
#include "app/core1.h"
#include "app/firmware_api.h"
#include "app/multicore.h"
#include "common/log.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "control/line_follower.h"
#include "control/robot.h"
#include "driver_lab/driver_lab.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/line_sensor.h"
#include "drivers/motor.h"
#include "drivers/tof.h"
#include "maze/maze.h"
#include "maze/mouse.h"

// ----------------------------------------------------------------------------
// Bluetooth UART as a stdio driver (used in DriverLab mode for printf mirroring)
// ----------------------------------------------------------------------------
static void uart_out_chars(const char* buf, int length)
{
    // Non-blocking mirror to uart0 (BT terminal at 9600 baud). Once the
    // 32-byte hardware FIFO is full we drop further chars rather than
    // stalling the 500 Hz control loop. The USB CDC sink (separate stdio
    // driver) and the dashboard's serial reader continue to receive every
    // byte at full fidelity. See CLAUDE.md "Loop dt".
    for (int i = 0; i < length; i++)
        if (uart_is_writable(uart0))
            uart_putc_raw(uart0, buf[i]);
}

static stdio_driver_t bt_driver = {
    .out_chars = uart_out_chars,
    .out_flush = nullptr,
    .in_chars  = nullptr,
    .next      = nullptr,
};

// ----------------------------------------------------------------------------
// Boot-time selection
// ----------------------------------------------------------------------------
static bool selectDriverLab(uint32_t timeout_ms)
{
    printf("\n==========================================\n");
    printf("  Jurababa -- boot\n");
    printf("==========================================\n");
    printf("  Press 'N' within %lu s for Normal CLI Mode.\n", timeout_ms / 1000);
    printf("  Otherwise DriverLab will start (default).\n");
    printf("==========================================\n\n");

    uint32_t start_ms        = to_ms_since_boot(get_absolute_time());
    int      countdown_print = -1;

    while (true)
    {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_ms;
        if (elapsed >= timeout_ms)
        {
            printf("\n*** DriverLab selected (default) ***\n\n");
            return true;
        }

        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
        {
            char ch = static_cast<char>(c);
            if (ch == 'N' || ch == 'n')
            {
                printf("\n*** Normal CLI Mode selected ***\n\n");
                return false;
            }
        }

        int seconds_left = static_cast<int>((timeout_ms - elapsed) / 1000);
        if (seconds_left != countdown_print && seconds_left >= 0)
        {
            printf("  %d...\n", seconds_left);
            countdown_print = seconds_left;
        }
    }
}

static SensorMode selectSensorMode(uint32_t timeout_ms)
{
    printf("\n==========================================\n");
    printf("  I2C0 sensor selection\n");
    printf("==========================================\n");
    printf("  Press 'T' for ToF (default), 'L' for LineSensor.\n\n");

    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (true)
    {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_ms;
        if (elapsed >= timeout_ms)
        {
            printf("\n*** ToF selected (default) ***\n\n");
            return SensorMode::TOF;
        }

        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
        {
            char ch = static_cast<char>(c);
            if (ch == 'T' || ch == 't')
            {
                printf("\n*** ToF selected ***\n\n");
                return SensorMode::TOF;
            }
            if (ch == 'L' || ch == 'l')
            {
                printf("\n*** LineSensor selected ***\n\n");
                return SensorMode::LINE_SENSOR;
            }
        }
    }
}

// ----------------------------------------------------------------------------
// DriverLab mode (single-core motor characterization)
// ----------------------------------------------------------------------------
static void runDriverLabMode(Battery* battery)
{
    // 9600 baud matches the HC-05 Bluetooth module. The CSV mirror in
    // uart_out_chars is non-blocking, so this slow baud cannot stall the
    // 500 Hz control loop — chars overflow the FIFO and are dropped on the
    // BT side instead. The USB CDC sink keeps full fidelity for the
    // dashboard. See CLAUDE.md "Loop dt".
    uart_init(uart0, 9600);
    gpio_set_function(PIN_BT_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_BT_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    stdio_set_driver_enabled(&bt_driver, true);

    printf("\n=== DriverLab ===\n");
    printf("Battery: %.2f V\n", battery->voltage());

    SensorMode sensor_mode = selectSensorMode(3000);

    Encoder left_encoder(pio0, PIN_ENCODER_L_A, false);
    Encoder right_encoder(pio0, PIN_ENCODER_R_A, true);
    Motor   left_motor(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, true);
    Motor   right_motor(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, true);
    IMU     imu(PIN_IMU_RX);

    // DriverLab is standalone — no Drivetrain, no Robot. Drivers only.
    // LineSensor and ToFs share I2C0 — exactly one branch runs.
    ToF*        left_tof_p    = nullptr;
    ToF*        front_tof_p   = nullptr;
    ToF*        right_tof_p   = nullptr;
    LineSensor* line_sensor_p = nullptr;

    if (sensor_mode == SensorMode::LINE_SENSOR)
    {
        static LineSensor line_sensor(i2c0, PIN_LINE_SDA, PIN_LINE_SCL);
        line_sensor.begin();
        line_sensor_p = &line_sensor;
        printf("LineSensor on I2C0\n");
    }
    else
    {
        static ToF left_tof(PIN_TOF_LEFT_XSHUT, 'L');
        static ToF front_tof(PIN_TOF_FRONT_XSHUT, 'F');
        static ToF right_tof(PIN_TOF_RIGHT_XSHUT, 'R');
        left_tof_p  = &left_tof;
        front_tof_p = &front_tof;
        right_tof_p = &right_tof;
        printf("ToFs on I2C0\n");
    }

    DriverLab driverlab(&left_motor, &right_motor, &left_encoder, &right_encoder, &imu, battery,
                        left_tof_p, front_tof_p, right_tof_p, line_sensor_p);
    driverlab.init();

    printf("\nReady. Type '?' for help.\n\n");

    const uint32_t  LOOP_PERIOD_US = static_cast<uint32_t>(LOOP_INTERVAL_S * 1.0e6f);
    absolute_time_t next_tick      = make_timeout_time_us(LOOP_PERIOD_US);

    while (true)
    {
        battery->update();         // 500 Hz; 10-sample MA = 20 ms time constant
        imu.update();              // no-op for omega; reserved for future fusion
        driverlab.tick();          // advances active trial by exactly one 500 Hz step
        driverlab.processSerial(); // poll keyboard for new commands
        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, LOOP_PERIOD_US);
    }
}

// ----------------------------------------------------------------------------
// Unified CLI mode (ToF or LineSensor)
// ----------------------------------------------------------------------------
static void runCliMode(Battery* battery, SensorMode sensor_mode)
{
    Bluetooth bluetooth(uart0, 9600, PIN_BT_TX, PIN_BT_RX);
    bluetooth.init();
    bluetooth.write("=== Jurababa CLI ===\r\n");
    Log::setBluetoothInterface(&bluetooth);
    Log::setBluetoothEnabled(true);

    // Maze + virtual mouse + wall bridge live on Core 0 in both sub-modes.
    std::array<int, 2>              start_cell = {0, 0};
    std::vector<std::array<int, 2>> goal_cells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    static Maze                     maze(MAZE_SIZE, MAZE_SIZE);
    static Mouse                    mouse(start_cell, std::string("n"), goal_cells, &maze);
    static FirmwareApi              api(&mouse);

    // LineFollower stack — only built in LineSensor mode (Core 1 stays
    // dormant so this side owns the H-bridge).
    LineFollower* line_follower_ptr = nullptr;
    if (sensor_mode == SensorMode::LINE_SENSOR)
    {
        static LineSensor line_sensor(i2c0, PIN_LINE_SDA, PIN_LINE_SCL);
        line_sensor.begin();

        static Encoder left_encoder(pio0, PIN_ENCODER_L_A, false);
        static Encoder right_encoder(pio0, PIN_ENCODER_R_A, true);
        static Motor   left_motor(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, true);
        static Motor   right_motor(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, true);
        static IMU     imu(PIN_IMU_RX);

        static Drivetrain   drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder,
                                       battery);
        static LineFollower line_follower(&drivetrain, &line_sensor, &imu, battery);
        line_follower_ptr = &line_follower;

        printf("LineFollower stack ready (Core 0 owns motors).\n");
    }
    else
    {
        // ToF mode: hand the battery pointer to Core 1 *before* launch so
        // its Drivetrain can voltage-compensate from the first tick.
        SensorHub::init();
        core1Configure(battery);
        multicore_launch_core1(core1Entry);

        // Core 1 pushes 1 to the FIFO once Robot::reset() returns.
        multicore_fifo_pop_blocking();
        printf("Core 1 ready (Robot @ 500 Hz).\n");
    }

    Cli::Deps deps;
    deps.bluetooth     = &bluetooth;
    deps.battery       = battery;
    deps.line_follower = line_follower_ptr;
    deps.driver_lab    = nullptr; // DriverLab requires direct motor access; not in Cli mode.
    deps.maze          = &maze;
    deps.mouse         = &mouse;
    deps.api           = &api;
    deps.sensor_mode   = sensor_mode;
    deps.start_cell    = start_cell;
    deps.goal_cells    = goal_cells;

    Cli cli(deps);
    cli.greet();
    cli.loop(); // never returns
}

// ----------------------------------------------------------------------------
// Startup LED
// ----------------------------------------------------------------------------
static void blinkStartupLED(int count, uint32_t on_ms, uint32_t off_ms)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < count; i++)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        if (i < count - 1)
            sleep_ms(off_ms);
    }
}

// ----------------------------------------------------------------------------
// Entry
// ----------------------------------------------------------------------------
int main()
{
    stdio_init_all();
    blinkStartupLED(3, 150, 150);
    sleep_ms(2000); // wait for USB-CDC enumeration

    Battery battery(PIN_BATTERY_ADC, 10000.0f, 5100.0f);
    battery.begin();
    // Quick pre-fill so the boot diagnostic printf in runDriverLabMode/runCliMode
    // has a non-zero reading before the 500 Hz tick loop spins up. The per-tick
    // updater downstream takes over once the loop starts.
    for (int i = 0; i < 3; i++)
    {
        battery.update();
        sleep_ms(2);
    }

    // Always hand a live Battery* downstream so hot-plug works: if VBAT is
    // absent at boot but plugged in later, BAT and voltage compensation pick
    // it up automatically. Motor::set_motor_volts() already handles the
    // <1 V case with an edge-triggered fallback warning (motor.cpp), so no
    // boot-time latch is needed here.
    Battery* battery_ptr = &battery;
    if (battery.voltage() < 1.0f)
    {
        printf("Battery low/absent at boot (%.2f V). Hot-plug VBAT and BAT will pick it up.\n",
               battery.voltage());
    }

    if (selectDriverLab(3000))
    {
        runDriverLabMode(battery_ptr);
    }
    else
    {
        SensorMode sensor_mode = selectSensorMode(3000);
        runCliMode(battery_ptr, sensor_mode);
    }

    return 0;
}
