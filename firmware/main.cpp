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
#include <stdio.h>
#include <string>
#include <vector>

#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/cli.h"
#include "app/firmware_api.h"
#include "common/bluetooth_stdio.h"
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
//
// Software TX ring buffer sits between printf and uart0's 32-byte hardware
// FIFO. printf can produce instant bursts (multi-line help banners, JSON
// blobs, multi-row CSV flushes) much larger than the FIFO; without the ring,
// every byte past 32 in a single printf call was silently dropped. The ring
// absorbs the burst; bt_drain() pumps ring -> FIFO non-blockingly, called
// both opportunistically at the end of every printf and once per 500 Hz
// DriverLab tick to keep the FIFO topped off between printfs.
//
// Single-core context (DriverLab is Core 0 only, no UART IRQ wired in this
// mode) means the ring has exactly one writer and one reader in the same
// execution context, so no locking is needed. `volatile` is a hedge against
// future IRQ-driven drain.
// ----------------------------------------------------------------------------
static constexpr uint32_t BT_RING_SIZE               = 2048; // power of 2 -> AND-mask wrap
static constexpr uint32_t BT_LINE_BUFFER_SIZE        = 256;
static constexpr uint32_t BT_DEFAULT_CSV_INTERVAL_MS = 50;
static volatile uint32_t  bt_ring_head               = 0;
static volatile uint32_t  bt_ring_tail               = 0;
static volatile uint32_t  bt_ring_max_depth          = 0;
static volatile uint32_t  bt_ring_dropped_bytes      = 0;
static volatile uint32_t  bt_suppressed_csv_lines    = 0;
static volatile uint32_t  bt_csv_interval_ms         = BT_DEFAULT_CSV_INTERVAL_MS;
static char               bt_ring[BT_RING_SIZE];
static char               bt_line_buffer[BT_LINE_BUFFER_SIZE];
static uint32_t           bt_line_length        = 0;
static bool               bt_line_passthrough   = false;
static bool               bt_line_csv_candidate = false;
static bool               bt_line_has_comma     = false;
static uint32_t           bt_last_csv_emit_ms   = 0;

static uint32_t bt_ring_depth()
{
    return (bt_ring_head - bt_ring_tail) & (BT_RING_SIZE - 1);
}

static void bt_note_depth()
{
    const uint32_t depth = bt_ring_depth();
    if (depth > bt_ring_max_depth)
        bt_ring_max_depth = depth;
}

static void bt_ring_push_raw(const char* buf, int length)
{
    for (int i = 0; i < length; i++)
    {
        uint32_t next = (bt_ring_head + 1) & (BT_RING_SIZE - 1);
        if (next == bt_ring_tail)
        {
            bt_ring_dropped_bytes += static_cast<uint32_t>(length - i);
            return; // ring full -> drop. Better than blocking the 500 Hz loop.
        }
        bt_ring[bt_ring_head] = buf[i];
        bt_ring_head          = next;
        bt_note_depth();
    }
}

static void bt_drain()
{
    while (bt_ring_tail != bt_ring_head && uart_is_writable(uart0))
    {
        uart_putc_raw(uart0, bt_ring[bt_ring_tail]);
        bt_ring_tail = (bt_ring_tail + 1) & (BT_RING_SIZE - 1);
    }
}

static bool bt_line_starts_csv_data(char ch)
{
    return (ch >= '0' && ch <= '9') || ch == '-' || ch == '+';
}

static bool bt_should_emit_csv_line()
{
    const uint32_t interval_ms = bt_csv_interval_ms;
    if (interval_ms == 0)
        return true;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (bt_last_csv_emit_ms == 0 || now_ms - bt_last_csv_emit_ms >= interval_ms)
    {
        bt_last_csv_emit_ms = now_ms;
        return true;
    }

    bt_suppressed_csv_lines++;
    return false;
}

static void bt_flush_line_buffer()
{
    if (bt_line_length == 0)
        return;

    const bool csv_data_line = bt_line_csv_candidate && bt_line_has_comma;
    if (!csv_data_line || bt_should_emit_csv_line())
        bt_ring_push_raw(bt_line_buffer, static_cast<int>(bt_line_length));

    bt_line_length        = 0;
    bt_line_passthrough   = false;
    bt_line_csv_candidate = false;
    bt_line_has_comma     = false;
}

static void bt_process_out_char(char ch)
{
    if (bt_line_passthrough)
    {
        bt_ring_push_raw(&ch, 1);
        if (ch == '\n' || ch == '\r')
            bt_line_passthrough = false;
        return;
    }

    if (bt_line_length == 0)
    {
        bt_line_csv_candidate = bt_line_starts_csv_data(ch);
        bt_line_has_comma     = false;
        if (!bt_line_csv_candidate)
        {
            bt_line_passthrough = true;
            bt_ring_push_raw(&ch, 1);
            if (ch == '\n' || ch == '\r')
                bt_line_passthrough = false;
            return;
        }
    }

    if (bt_line_length < BT_LINE_BUFFER_SIZE)
    {
        bt_line_buffer[bt_line_length++] = ch;
        if (ch == ',')
            bt_line_has_comma = true;
    }

    if (ch == '\n' || ch == '\r' || bt_line_length >= BT_LINE_BUFFER_SIZE)
        bt_flush_line_buffer();
}

static void uart_out_chars(const char* buf, int length)
{
    for (int i = 0; i < length; i++)
        bt_process_out_char(buf[i]);
    bt_drain(); // small printfs that fit in the FIFO go straight through
}

static stdio_driver_t bt_driver = {
    .out_chars = uart_out_chars,
    .out_flush = nullptr,
    .in_chars  = nullptr,
    .next      = nullptr,
};

namespace BluetoothStdio
{
void setCsvIntervalMs(uint32_t interval_ms)
{
    bt_csv_interval_ms  = interval_ms;
    bt_last_csv_emit_ms = 0;
}

uint32_t csvIntervalMs()
{
    return bt_csv_interval_ms;
}

Diagnostics diagnostics()
{
    Diagnostics diag;
    diag.ring_size            = BT_RING_SIZE;
    diag.depth                = bt_ring_depth();
    diag.max_depth            = bt_ring_max_depth;
    diag.dropped_bytes        = bt_ring_dropped_bytes;
    diag.suppressed_csv_lines = bt_suppressed_csv_lines;
    diag.csv_interval_ms      = bt_csv_interval_ms;
    return diag;
}

void resetDiagnostics()
{
    bt_ring_max_depth       = bt_ring_depth();
    bt_ring_dropped_bytes   = 0;
    bt_suppressed_csv_lines = 0;
    bt_last_csv_emit_ms     = 0;
}
} // namespace BluetoothStdio

// ----------------------------------------------------------------------------
// Boot-time selection
// ----------------------------------------------------------------------------
static bool selectDriverLab(uint32_t timeout_ms)
{
    printf("\n==========================================\n");
    printf("  Jurababa -- boot\n");
    printf("==========================================\n");
    printf("  Press 'M' within %lu s for DriverLab Mode.\n", timeout_ms / 1000);
    printf("  Otherwise Normal CLI will start (default).\n");
    printf("==========================================\n\n");

    uint32_t start_ms        = to_ms_since_boot(get_absolute_time());
    int      countdown_print = -1;

    while (true)
    {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_ms;
        if (elapsed >= timeout_ms)
        {
            printf("\n*** Normal CLI Mode selected (default) ***\n\n");
            return false;
        }

        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
        {
            char ch = static_cast<char>(c);
            if (ch == 'M' || ch == 'm')
            {
                printf("\n*** DriverLab selected ***\n\n");
                return true;
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
    // 115200 baud matches the HC-05 Bluetooth module. The HC-05 must be
    // reconfigured once via AT mode to use this rate (default ships at
    // 9600): hold KEY high while powering on, open a terminal at 38400
    // (HC-05 AT mode is always 38400), and send `AT+UART=115200,0,0`. At
    // 9600, the BT link only carries ~960 B/s versus the reporter's
    // ~3-9 KB/s peak, so the lossy uart_out_chars dropped trailing bytes
    // (often the row-terminating '\n') and CSV rows arrived smushed
    // together. See CLAUDE.md "Loop dt".
    uart_init(uart0, 115200);
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
        bt_drain();                // pump TX ring -> uart0 FIFO between printfs
        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, LOOP_PERIOD_US);
    }
}

// ----------------------------------------------------------------------------
// Unified CLI mode (ToF or LineSensor)
// ----------------------------------------------------------------------------
static void runCliMode(Battery* battery, SensorMode sensor_mode)
{
    Bluetooth bluetooth(uart0, 115200, PIN_BT_TX, PIN_BT_RX);
    bluetooth.init();
    bluetooth.write("=== Jurababa CLI ===\r\n");
    bluetooth.drain();
    Log::setBluetoothInterface(&bluetooth);
    Log::setBluetoothPriority(LogPriority::INFO);
    Log::setBluetoothEnabled(true);

    // Maze + virtual mouse + wall bridge live on Core 0 in both sub-modes.
    std::array<int, 2>              start_cell = {0, 0};
    std::vector<std::array<int, 2>> goal_cells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    static Maze                     maze(MAZE_SIZE, MAZE_SIZE);
    static Mouse                    mouse(start_cell, std::string("n"), goal_cells, &maze);
    static FirmwareApi              api(&mouse);
    api.setUp(start_cell, goal_cells);

    // LineFollower stack — only built in LineSensor mode (Robot stays
    // out of the picture so LineFollower owns the H-bridge).
    LineFollower* line_follower_ptr = nullptr;
    Robot*        robot_ptr         = nullptr;
    ToF*          left_tof_ptr      = nullptr;
    ToF*          front_tof_ptr     = nullptr;
    ToF*          right_tof_ptr     = nullptr;

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

        printf("LineFollower stack ready.\n");
    }
    else
    {
        // ToF mode: single-core UKMARS shape. All hardware lives on Core 0;
        // Robot is advanced by a 500 Hz hardware-timer alarm registered
        // below. The CLI's main loop (cli.loop()) handles serial I/O and
        // polls ToFs at 50 Hz, exactly like mazerunner-core's Arduino
        // loop() / Timer2 split.
        static Encoder    left_encoder(pio0, PIN_ENCODER_L_A, false);
        static Encoder    right_encoder(pio0, PIN_ENCODER_R_A, true);
        static Motor      left_motor(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, true);
        static Motor      right_motor(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, true);
        static IMU        imu(PIN_IMU_RX);
        static ToF        left_tof(PIN_TOF_LEFT_XSHUT, 'L');
        static ToF        front_tof(PIN_TOF_FRONT_XSHUT, 'F');
        static ToF        right_tof(PIN_TOF_RIGHT_XSHUT, 'R');
        static Drivetrain drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder,
                                     battery);
        static Robot      robot(&drivetrain, &imu);

        api.setRobot(&robot);
        api.setTofSensors(&left_tof, &front_tof, &right_tof);
        robot_ptr     = &robot;
        left_tof_ptr  = &left_tof;
        front_tof_ptr = &front_tof;
        right_tof_ptr = &right_tof;

        // Hand robot + ToFs to a 500 Hz timer alarm. Mirrors UKMARS
        // systick.h: encoders.update → motion.update → motors.update_controllers
        // (we do all of that inside Robot::update). 50 Hz ToF poll runs
        // outside the alarm to keep its budget sub-millisecond.
        struct CoreLoopCtx
        {
            Robot* robot;
            ToF*   left_tof;
            ToF*   front_tof;
            ToF*   right_tof;
        };
        static CoreLoopCtx ctx{&robot, &left_tof, &front_tof, &right_tof};

        static repeating_timer_t systick_timer;
        add_repeating_timer_us(
            -2000 /* 500 Hz, negative => fire on absolute schedule */,
            +[](repeating_timer_t* t) -> bool
            {
                auto* c = static_cast<CoreLoopCtx*>(t->user_data);
                c->robot->update();
                return true;
            },
            &ctx, &systick_timer);

        printf("Robot @ 500 Hz on Core 0 (single-core UKMARS shape).\n");
    }

    Cli::Deps deps;
    deps.bluetooth     = &bluetooth;
    deps.battery       = battery;
    deps.robot         = robot_ptr;
    deps.left_tof      = left_tof_ptr;
    deps.front_tof     = front_tof_ptr;
    deps.right_tof     = right_tof_ptr;
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
