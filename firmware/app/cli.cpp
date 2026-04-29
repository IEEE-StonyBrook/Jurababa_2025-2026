#include "app/cli.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/commands.h"
#include "app/motion_state.h"
#include "app/multicore.h"
#include "app/start_gesture.h"
#include "common/log.h"
#include "control/line_follower.h"
#include "driver_lab/driver_lab.h"
#include "drivers/battery.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/flood_fill.h"
#include "navigation/path_utils.h"

namespace
{
bool isAllDigits(const char* s)
{
    if (s == nullptr || *s == '\0')
        return false;
    if (*s == '-' || *s == '+')
        ++s;
    if (*s == '\0')
        return false;
    for (; *s; ++s)
        if (!std::isdigit(static_cast<unsigned char>(*s)))
            return false;
    return true;
}
} // namespace

Cli::Cli(const Deps& deps) : deps_(deps)
{
    // Wire ourselves into the API as the motion-complete waiter so every
    // CommandHub::send blocks here until Core 1 reports done.
    if (deps_.api != nullptr)
        deps_.api->setMotionWaiter(this);
}

void Cli::greet()
{
    printf("\n");
    printf("==========================================\n");
    printf("  Jurababa Micromouse — UKMARS-style CLI \n");
    printf("==========================================\n");
    printf("Sensor mode: %s\n", deps_.sensor_mode == SensorMode::TOF ? "ToF" : "LineSensor");
    printf("Type a number to run a function, or '?' for help.\n");
    printf("Numbered functions:\n");
    printf("   0  STOP / halt\n");
    printf("   1  Sensor snapshot\n");
    printf("   2  Explore (FloodFill)            [TOF]\n");
    printf("   3  Return to start                [TOF]\n");
    printf("   4  Speed run (A* + diagonals)     [TOF]\n");
    printf("   5  SS90 right turn (90 deg)\n");
    printf("   6  Smooth right 90 turn\n");
    printf("   7  Forward 1 cell (encoder cal)\n");
    printf("   8  Line follow                    [LINE]\n");
    printf("   9  TURN-OL (DriverLab IMU spin)\n");
    printf("  10  Battery voltage\n");
    printf("Short commands: ?  B (battery)  X (halt)  RUN n  G (start)\n");
    if (deps_.driver_lab != nullptr)
        printf("Alpha tokens (OL, STEP, EXPORT, MOVE, …) are forwarded to DriverLab.\n");
    else
        printf("Reboot in DriverLab mode (press 'M') for OL/STEP/EXPORT/MOVE/TURN trials.\n");
    printf("\n");
}

void Cli::loop()
{
    while (true)
    {
        pollOnce();
        sleep_ms(2);
    }
}

bool Cli::pollOnce()
{
    handleBluetoothCommand();

    bool processed = false;
    if (readLineNonBlocking())
    {
        executeLine();
        processed = true;
    }
    return processed;
}

bool Cli::readLineNonBlocking()
{
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT || c < 0)
        return false;

    char ch = static_cast<char>(c);
    if (ch == '\r' || ch == '\n')
    {
        if (line_index_ == 0)
            return false; // ignore bare CR/LF
        line_buffer_[line_index_] = '\0';
        line_index_               = 0;
        return true;
    }

    if (line_index_ < LINE_BUFFER_SIZE - 1)
    {
        line_buffer_[line_index_++] = ch;
        line_buffer_[line_index_]   = '\0';
    }
    return false;
}

void Cli::executeLine()
{
    // Strip leading whitespace.
    char* p = line_buffer_;
    while (*p == ' ' || *p == '\t')
        ++p;
    if (*p == '\0')
        return;

    // Capture the first whitespace-delimited token without destroying the
    // rest of the line — DriverLab needs the full line for its own tokenizer.
    char first_token[32];
    int  i = 0;
    while (p[i] != '\0' && p[i] != ' ' && p[i] != '\t' && i < 31)
    {
        first_token[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(p[i])));
        ++i;
    }
    first_token[i] = '\0';

    // Pure integer → numbered dispatch.
    if (isAllDigits(first_token))
    {
        int n = std::atoi(first_token);
        runFunction(n);
        last_function_ = n;
        return;
    }

    // Built-in short commands.
    if (std::strcmp(first_token, "?") == 0 || std::strcmp(first_token, "HELP") == 0)
    {
        greet();
        return;
    }
    if (std::strcmp(first_token, "X") == 0 || std::strcmp(first_token, "HALT") == 0)
    {
        runFunction(0);
        return;
    }
    if (std::strcmp(first_token, "B") == 0 || std::strcmp(first_token, "BAT") == 0 ||
        std::strcmp(first_token, "BATTERY") == 0)
    {
        runFunction(10);
        return;
    }
    if (std::strcmp(first_token, "G") == 0 || std::strcmp(first_token, "GO") == 0)
    {
        // Re-run the last numbered function (USB analog of Bluetooth START).
        if (last_function_ >= 0)
            runFunction(last_function_);
        else
            printf("No previous function to re-run.\n");
        return;
    }
    if (std::strcmp(first_token, "RUN") == 0)
    {
        // RUN n — same as typing the number.
        char* arg = p + std::strlen(first_token);
        while (*arg == ' ' || *arg == '\t')
            ++arg;
        if (isAllDigits(arg))
        {
            int n = std::atoi(arg);
            runFunction(n);
            last_function_ = n;
        }
        else
        {
            printf("RUN expects an integer argument.\n");
        }
        return;
    }

    // Anything else: forward the original line to DriverLab unchanged so
    // OL, STEP, EXPORT, MOVE, TURN, … keep working with their existing
    // arg parser. We hand `p` (post-whitespace-strip) — DriverLab doesn't
    // mind leading-stripped lines.
    if (deps_.driver_lab != nullptr)
        deps_.driver_lab->executeLine(p);
    else
        printf("Unknown command: %s\n", first_token);
}

void Cli::handleBluetoothCommand()
{
    Bluetooth* bt = deps_.bluetooth;
    if (bt == nullptr || !bt->hasCommand())
        return;

    Bluetooth::Command cmd = bt->command();
    switch (cmd)
    {
        case Bluetooth::Command::START:
            if (last_function_ >= 0)
            {
                LOG_INFO("BT START: re-running function " << last_function_);
                runFunction(last_function_);
            }
            else
            {
                bt->write("BT START: no last function. Type a number first.\r\n");
            }
            break;
        case Bluetooth::Command::HALT:
            LOG_INFO("BT HALT");
            runFunction(0);
            break;
        case Bluetooth::Command::RESET:
            // Full Mouse/Maze reset is a reboot operation — the brain owns
            // wall state, and reseeding it mid-session has subtle ordering
            // requirements. Keep RESET as STOP + reboot hint for now.
            LOG_INFO("BT RESET: stopping. Reboot for full Maze reset.");
            runFunction(0);
            break;
        case Bluetooth::Command::BATTERY:
            runFunction(10);
            break;
        default:
            break;
    }
}

void Cli::pollHaltOnly()
{
    // USB X / x.
    int c = getchar_timeout_us(0);
    if (c >= 0)
    {
        char ch = static_cast<char>(c);
        if (ch == 'X' || ch == 'x')
        {
            CommandHub::send(CommandType::STOP);
            halted_ = true;
            return;
        }
    }

    // Bluetooth HALT.
    Bluetooth* bt = deps_.bluetooth;
    if (bt != nullptr && bt->hasCommand())
    {
        Bluetooth::Command cmd = bt->command();
        if (cmd == Bluetooth::Command::HALT)
        {
            CommandHub::send(CommandType::STOP);
            halted_ = true;
        }
        // Other commands are dropped here — they're handled by the main
        // loop's `handleBluetoothCommand()`. During waitForMotionComplete
        // we don't want a stray BATTERY query to derail the motion.
    }
}

void Cli::waitForMotionComplete()
{
    // Worst-case wakeup latency is `sleep_ms(2)` + one Core-1 tick (2 ms),
    // so HALT cuts through within ~4 ms. Good enough for human reaction
    // times and well under the 8-deep FIFO's drain rate.
    while (MotionState::active)
    {
        pollHaltOnly();
        if (halted_)
        {
            // Drop the flag so we exit cleanly even if Core 1 hasn't
            // processed the STOP yet.
            MotionState::active = false;
            return;
        }
        sleep_ms(2);
    }
}

void Cli::runFunction(int n)
{
    halted_ = false;

    auto needsTof = [&](const char* what)
    {
        if (deps_.sensor_mode != SensorMode::TOF)
        {
            printf("%s requires ToF sensor mode. Reboot and select 'T'.\n", what);
            return false;
        }
        return true;
    };

    auto startWithGesture = [&](bool tof_available) -> bool
    {
        printf("Waiting for start gesture (wave hand / send 'G' / BT START)…\n");
        StartTrigger t = waitForStartGesture(deps_.bluetooth, tof_available);
        if (t == StartTrigger::CANCELLED)
        {
            printf("Cancelled.\n");
            halted_ = true;
            return false;
        }
        return true;
    };

    switch (n)
    {
        case 0:
        {
            CommandHub::send(CommandType::STOP);
            // Drop the in-flight flag so any Core-0 spin in waitForMotionComplete
            // unblocks immediately rather than waiting on Core 1's profile coast.
            MotionState::active = false;
            halted_             = true;
            printf("STOP\n");
            break;
        }

        case 1:
        {
            dumpSensorsOneShot();
            break;
        }

        case 2:
        {
            if (!needsTof("Explore (FloodFill)"))
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.api != nullptr)
                deps_.api->setPhaseColor('y');
            printf("Exploring…\n");
            FloodFill::explore(*deps_.mouse, *deps_.api, /*diagonals=*/false);
            printf("Explore done.\n");
            break;
        }

        case 3:
        {
            if (!needsTof("Return to start"))
                break;
            if (!startWithGesture(true))
                break;
            printf("Returning to start…\n");
            PathUtils::setAllExplored(deps_.mouse);
            std::vector<std::array<int, 2>> goals = {deps_.start_cell};
            PathUtils::traversePath(deps_.api, deps_.mouse, goals,
                                    /*diagonals=*/false, /*all_explored=*/true,
                                    /*avoid_goals=*/false);
            printf("Return done.\n");
            break;
        }

        case 4:
        {
            if (!needsTof("Speed run"))
                break;
            if (!startWithGesture(true))
                break;
            printf("Speed run (A* + diagonals)…\n");
            PathUtils::traversePath(deps_.api, deps_.mouse, deps_.goal_cells,
                                    /*diagonals=*/true, /*all_explored=*/true,
                                    /*avoid_goals=*/false);
            printf("Speed run done.\n");
            break;
        }

        case 5:
        {
            if (!startWithGesture(deps_.sensor_mode == SensorMode::TOF))
                break;
            // 2 × 45° steps = 90° (turn-unit fix in core1.cpp).
            CommandHub::send(CommandType::TURN_RIGHT, 2);
            waitForMotionComplete();
            printf("SS90 right done.\n");
            break;
        }

        case 6:
        {
            if (!startWithGesture(deps_.sensor_mode == SensorMode::TOF))
                break;
            CommandHub::send(CommandType::ARC_TURN_RIGHT_90);
            waitForMotionComplete();
            printf("Smooth 90 right done.\n");
            break;
        }

        case 7:
        {
            if (!startWithGesture(deps_.sensor_mode == SensorMode::TOF))
                break;
            SensorData before;
            SensorHub::snapshot(before);
            CommandHub::send(CommandType::MOVE_FWD, 1);
            waitForMotionComplete();
            SensorData after;
            SensorHub::snapshot(after);
            printEncoderDelta(before.left_encoder, before.right_encoder, after.left_encoder,
                              after.right_encoder);
            break;
        }

        case 8:
        {
            if (deps_.sensor_mode != SensorMode::LINE_SENSOR)
            {
                printf("Line follow requires LineSensor mode. Reboot and select 'L'.\n");
                break;
            }
            if (deps_.line_follower == nullptr)
            {
                printf("Line follower not constructed.\n");
                break;
            }
            // Skip ToF gesture (front ToF isn't available); 'G' / BT START only.
            if (!startWithGesture(false))
                break;
            runLineFollowEventLoop();
            break;
        }

        case 9:
        {
            if (deps_.driver_lab != nullptr)
                deps_.driver_lab->executeLine("TURNOL");
            else
                printf("DriverLab not initialized.\n");
            break;
        }

        case 10:
        {
            if (deps_.battery == nullptr)
            {
                printf("Battery monitor not initialized.\n");
                break;
            }
            deps_.battery->update();
            printf("Battery: %.2f V\n", deps_.battery->voltage());
            break;
        }

        default:
        {
            printf("Unknown function: %d. STOP-ing for safety.\n", n);
            CommandHub::send(CommandType::STOP);
            halted_ = true;
            break;
        }
    }
}

void Cli::dumpSensorsOneShot()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    printf("L=%d mm  F=%d mm  R=%d mm  yaw=%.1f deg  encL=%ld encR=%ld\n", snap.tof_left_mm,
           snap.tof_front_mm, snap.tof_right_mm, static_cast<double>(snap.imu_yaw),
           static_cast<long>(snap.left_encoder), static_cast<long>(snap.right_encoder));
    if (deps_.battery != nullptr)
    {
        deps_.battery->update();
        printf("Battery: %.2f V\n", deps_.battery->voltage());
    }
}

void Cli::printEncoderDelta(int32_t l_before, int32_t r_before, int32_t l_after, int32_t r_after)
{
    int32_t dl = l_after - l_before;
    int32_t dr = r_after - r_before;
    printf("Encoder delta over 1 cell:  L=%ld ticks   R=%ld ticks\n", static_cast<long>(dl),
           static_cast<long>(dr));
}

void Cli::runLineFollowEventLoop()
{
    // Body lifted from the pre-refactor `runLineFollowingMode`. The intersection
    // event queue is hardcoded here so this function is self-contained — once
    // the CLI grows a `LINE-QUEUE` command we can pull it from input instead.
    LineFollower* lf = deps_.line_follower;
    if (lf == nullptr)
        return;

    const char  default_queue[] = "L#F#R#";
    const char* events          = default_queue;
    char        parsed[32]      = {};
    int         parsed_len      = 0;
    for (int i = 0; events[i] != '\0' && parsed_len < 31; ++i)
    {
        char ch = events[i];
        if (ch == 'L' || ch == 'l' || ch == 'F' || ch == 'f' || ch == 'R' || ch == 'r')
            parsed[parsed_len++] = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    parsed[parsed_len] = '\0';

    int queue_index = 0;
    printf("Line follow starting. Queue: %s (%d events)\n", parsed, parsed_len);
    lf->startFollowing();

    const uint32_t  CONTROL_PERIOD_US = 2000; // 500 Hz
    absolute_time_t next_tick         = make_timeout_time_us(CONTROL_PERIOD_US);
    absolute_time_t last_tick         = get_absolute_time();

    while (!halted_)
    {
        // Halt and command-line responsiveness while the follower runs.
        pollHaltOnly();
        handleBluetoothCommand();

        absolute_time_t now = get_absolute_time();
        float           dt  = absolute_time_diff_us(last_tick, now) * 1e-6f;
        last_tick           = now;

        lf->update(dt);

        if (lf->isIntersectionDetected() && lf->isMotionDone())
        {
            if (queue_index < parsed_len)
            {
                char action = parsed[queue_index++];
                printf("Intersection: %c (%d/%d)\n", action, queue_index, parsed_len);
                switch (action)
                {
                    case 'L':
                        lf->turnLeft90();
                        break;
                    case 'R':
                        lf->turnRight90();
                        break;
                    case 'F':
                    default:
                        break;
                }
            }
            else
            {
                printf("Queue empty — continuing forward.\n");
            }
        }

        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, CONTROL_PERIOD_US);
    }

    printf("Line follow halted.\n");
}
