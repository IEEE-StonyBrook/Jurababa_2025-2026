#include "app/cli.h"

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "pico/stdlib.h"

#include "app/bluetooth.h"
#include "app/bootsel_button.h"
#include "app/leds.h"
#include "app/mouse.h"
#include "app/start_gesture.h"
#include "common/log.h"
#include "common/tof_wall_utils.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "config/smooth_turn.h"
#include "control/line_follower.h"
#include "control/motion.h"
#include "drivers/battery.h"
#include "drivers/beacon_ir.h"
#include "drivers/tof.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/path_utils.h"

namespace
{
constexpr char kBackspace = 0x08;

const char* movementStyleName(Mouse::MovementStyle style)
{
    return style == Mouse::MovementStyle::Smooth ? "SMOOTH" : "STATIONARY";
}

std::string coordinateText(const std::array<int, 2>& cell)
{
    return "(" + std::to_string(cell[0]) + "," + std::to_string(cell[1]) + ")";
}

std::string goalsText(const std::vector<std::array<int, 2>>& goals)
{
    std::string text;
    for (size_t i = 0; i < goals.size(); ++i)
    {
        if (i > 0)
            text += ",";
        text += coordinateText(goals[i]);
    }
    return text.empty() ? "(none)" : text;
}

std::string linePathsText(uint8_t paths_mask)
{
    std::string text;
    if ((paths_mask & LineSensor::PATH_LEFT) != 0)
        text += 'L';
    if ((paths_mask & LineSensor::PATH_FORWARD) != 0)
        text += 'F';
    if ((paths_mask & LineSensor::PATH_RIGHT) != 0)
        text += 'R';
    return text.empty() ? "-" : text;
}

const char* yesNo(bool value)
{
    return value ? "yes" : "no";
}

std::string hexByteText(uint8_t value)
{
    char buffer[5];
    std::snprintf(buffer, sizeof(buffer), "0x%02X", value);
    return buffer;
}

std::string lineBitsText(uint8_t active_mask)
{
    std::string text;
    text.reserve(LINE_SENSOR_COUNT);
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i)
        text += ((active_mask & (1u << i)) != 0) ? '#' : '.';
    return text;
}

std::string lineRouteProgressText(const LineFollower* follower)
{
    if (follower == nullptr)
        return "route=- next=0/0 next_cmd=-";

    const char*   route        = follower->route();
    const size_t  route_length = route != nullptr ? std::strlen(route) : 0;
    const uint8_t route_index  = follower->routeIndex();
    const char    next_command = route_index < route_length ? route[route_index] : '-';

    return std::string("route=") + (route_length > 0 ? route : "-") +
           " next=" + std::to_string(static_cast<unsigned>(route_index)) + "/" +
           std::to_string(route_length) + " next_cmd=" + next_command;
}

std::string lineLastDecisionText(const LineFollower* follower)
{
    if (follower == nullptr || !follower->lastIntersectionValid())
        return "last: none";

    return "last: paths=" + linePathsText(follower->lastIntersectionPathsMask()) +
           " cmd=" + std::string(1, follower->lastRouteCommand()) +
           " match=" + yesNo(follower->lastRouteChoiceMatched()) +
           " peak=" + hexByteText(follower->lastIntersectionRawPeakMask()) +
           " dt=" + std::to_string(follower->lastIntersectionElapsedMs()) + "ms";
}

int lineRouteCommandCode(char command)
{
    switch (command)
    {
        case 'L':
            return -1;
        case 'F':
            return 0;
        case 'R':
            return 1;
        default:
            return 9;
    }
}

const char* mazeMaskName(MazeMask mask)
{
    return mask == MASK_CLOSED ? "CLOSED" : "OPEN";
}

bool startsNumericArg(const char* text)
{
    if (text == nullptr || *text == '\0')
        return false;
    if (*text == '+' || *text == '-')
        ++text;
    return std::isdigit(static_cast<unsigned char>(*text)) || *text == '.';
}

bool equalsIgnoreCase(const char* lhs, const char* rhs)
{
    if (lhs == nullptr || rhs == nullptr)
        return false;

    while (*lhs != '\0' && *rhs != '\0')
    {
        const char l = static_cast<char>(std::toupper(static_cast<unsigned char>(*lhs)));
        const char r = static_cast<char>(std::toupper(static_cast<unsigned char>(*rhs)));
        if (l != r)
            return false;
        ++lhs;
        ++rhs;
    }
    return *lhs == '\0' && *rhs == '\0';
}

bool parsePathSmoothFlag(const char* text, bool& smooth_turns)
{
    if (equalsIgnoreCase(text, "1") || equalsIgnoreCase(text, "TRUE") ||
        equalsIgnoreCase(text, "SMOOTH"))
    {
        smooth_turns = true;
        return true;
    }
    if (equalsIgnoreCase(text, "0") || equalsIgnoreCase(text, "FALSE") ||
        equalsIgnoreCase(text, "STILL") || equalsIgnoreCase(text, "STATIONARY"))
    {
        smooth_turns = false;
        return true;
    }
    return false;
}

bool parsePathOption(const char* text, bool& smooth_turns, bool& raw_mode)
{
    if (equalsIgnoreCase(text, "RAW"))
    {
        raw_mode = true;
        return true;
    }
    if (equalsIgnoreCase(text, "POSE") || equalsIgnoreCase(text, "HOLD") ||
        equalsIgnoreCase(text, "HEADING"))
    {
        raw_mode = false;
        return true;
    }
    return parsePathSmoothFlag(text, smooth_turns);
}

bool parseFloatArg(const Args& args, int index, float min_value, float max_value, float& value)
{
    if (index < 0 || index >= args.argc)
        return false;

    char* end    = nullptr;
    float parsed = std::strtof(args.argv[index], &end);
    if (end == args.argv[index] || *end != '\0')
        return false;
    if (parsed < min_value || parsed > max_value)
        return false;

    value = parsed;
    return true;
}

float normalizeYawDelta(float delta)
{
    while (delta > 180.0f)
        delta -= 360.0f;
    while (delta < -180.0f)
        delta += 360.0f;
    return delta;
}

constexpr float PATH_DEFAULT_SPEED_MMPS      = CLI_PATH_SPEED_MMPS;
constexpr float PATH_DEFAULT_ACCEL_MMPS2     = CLI_PATH_ACCEL_MMPS2;
constexpr float PATH_DEFAULT_OMEGA_DEGPS     = CLI_PATH_OMEGA_DEGPS;
constexpr float PATH_DEFAULT_ALPHA_DEGPS2    = CLI_PATH_ALPHA_DEGPS2;
constexpr float CLI_PATH_NO_SPEED_LIMIT_MMPS = std::numeric_limits<float>::max();
constexpr float CLI_PATH_MAX_ACCEL_MMPS2     = 5000.0f;
constexpr float CLI_PATH_MAX_OMEGA_DEGPS     = 720.0f;
constexpr float CLI_PATH_MAX_ALPHA_DEGPS2    = 7200.0f;

} // namespace

CommandLineInterface* CommandLineInterface::s_instance_ = nullptr;

bool CommandLineInterface::haltCheckThunk()
{
    if (s_instance_ == nullptr)
        return false;
    s_instance_->pollHaltOnly();
    return s_instance_->halted_;
}

StartTrigger CommandLineInterface::compServiceThunk(void* ctx)
{
    auto* self = static_cast<CommandLineInterface*>(ctx);
    if (self == nullptr)
        return StartTrigger::NONE;
    return self->compServiceTick();
}

StartTrigger CommandLineInterface::compServiceTick()
{
    // Run the full CLI input pipeline so diagnostic commands (W/C/D/S/E/B,
    // F<n>, etc.) work normally while COMP is gesture-armed. process_serial_data
    // handles serial char accumulation, line-end dispatch into run_short_cmd /
    // run_long_cmd, and Bluetooth command intake. Single-letter shortcuts
    // G/H/J set comp_pending_trigger_ via run_short_cmd; Bluetooth START
    // does the same via handleBluetoothCommand.
    process_serial_data();

    if (comp_pending_trigger_ != StartTrigger::NONE)
    {
        const StartTrigger t  = comp_pending_trigger_;
        comp_pending_trigger_ = StartTrigger::NONE;
        return t;
    }

    if (halted_)
    {
        halted_ = false;
        return StartTrigger::CANCELLED;
    }

    return StartTrigger::NONE;
}

CommandLineInterface::CommandLineInterface(const Deps& deps) : deps_(deps)
{
    s_instance_ = this;
    if (deps_.mouse != nullptr)
        deps_.mouse->setHaltCheck(&CommandLineInterface::haltCheckThunk);
}

void CommandLineInterface::print(const char* text)
{
    if (text == nullptr)
        return;

    std::fputs(text, stdout);
    if (deps_.bluetooth != nullptr)
    {
        deps_.bluetooth->write(text);
        drainConsole();
    }
}

void CommandLineInterface::print(const std::string& text)
{
    print(text.c_str());
}

void CommandLineInterface::printFormat(const char* format, ...)
{
    if (format == nullptr)
        return;

    char    buffer[384];
    va_list args;
    va_start(args, format);
    int length = std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
        return;

    if (static_cast<size_t>(length) < sizeof(buffer))
    {
        print(buffer);
        return;
    }

    std::string dynamic_buffer(static_cast<size_t>(length) + 1, '\0');
    va_start(args, format);
    std::vsnprintf(&dynamic_buffer[0], dynamic_buffer.size(), format, args);
    va_end(args);
    dynamic_buffer.resize(static_cast<size_t>(length));
    print(dynamic_buffer);
}

void CommandLineInterface::printArgs(const Args& args)
{
    for (int i = 0; i < args.argc; ++i)
        printFormat("%s ", args.argv[i]);
    print("\n");
}

void CommandLineInterface::drainConsole()
{
    if (deps_.bluetooth == nullptr)
        return;

    Log::drainBluetooth();
    deps_.bluetooth->drain();
}

void CommandLineInterface::greet()
{
    print("\n");
    print("==========================================\n");
    print("  Jurababa Micromouse -- UKMARS CLI :)\n");
    print("==========================================\n");
    help();
    prompt();
}

void CommandLineInterface::loop()
{
    // UKMARS-style main loop. Three concurrent activities, all on Core 0:
    //   1. Serial input + command dispatch (process_serial_data → ~500 Hz)
    //   2. Sensor service at its configured cadence
    //   3. Battery filter update at 500 Hz (cheap; ADC is non-blocking)
    //
    // Motion::update() runs concurrently via the 500 Hz hardware timer
    // alarm registered in main.cpp for ToF mode. LineSensor mode has no
    // timer; the CLI loop advances LineFollower + Motion on this same 500 Hz
    // cadence.
    const uint32_t  loop_period_us = static_cast<uint32_t>(LOOP_INTERVAL_S * 1.0e6f);
    absolute_time_t next_tick      = make_timeout_time_us(loop_period_us);
    while (true)
    {
        process_serial_data();
        if (deps_.battery != nullptr)
            deps_.battery->update();
        if (deps_.mouse != nullptr)
            deps_.mouse->serviceSensors();
        if (deps_.sensor_mode == SensorMode::LINE_SENSOR && deps_.line_follower != nullptr)
        {
            deps_.line_follower->update(LOOP_INTERVAL_S);
            if (deps_.motion != nullptr)
                deps_.motion->update();

            // LINE telemetry + button poll — only while actively following.
            // The follower's other states (Idle/Stopping/Turning) are short or
            // already loud enough on their own log lines, so we keep the
            // periodic emission scoped to the running phase.
            const bool line_running =
                deps_.line_follower->state() == LineFollower::State::FollowingLine;
            if (line_running)
            {
                if (!line_running_was_active_)
                {
                    // Edge: just entered running state from a turn or fresh
                    // START. Reset pacing so the first telemetry line lands
                    // ~kLineTelemetryTicks into the run, not immediately.
                    line_telemetry_counter_   = 0;
                    line_button_poll_counter_ = 0;
                    line_running_was_active_  = true;
                }
                if (++line_telemetry_counter_ >= kLineTelemetryTicks)
                {
                    line_telemetry_counter_ = 0;
                    printLineRunningTelemetry();
                }
                // BOOTSEL poll at ~10 Hz (50 ticks @ 500 Hz). Reading the
                // BOOTSEL pin briefly suspends XIP, so we keep the cadence
                // low. A 100 ms latency on a panic-stop press is fine.
                if (++line_button_poll_counter_ >= 50)
                {
                    line_button_poll_counter_ = 0;
                    if (abortRequested())
                    {
                        printFormat("LINE BOOTSEL pressed -> stopping run.\n");
                        printLineRunSummary();
                        deps_.line_follower->stop();
                        // Wait for release so the next LINE START doesn't
                        // immediately consume this still-held press.
                        while (abortRequested())
                            sleep_ms(20);
                        sleep_ms(150);
                    }
                }
            }
            else if (line_running_was_active_)
            {
                // Edge: just left running state (turn, intersection, stop).
                line_running_was_active_  = false;
                line_telemetry_counter_   = 0;
                line_button_poll_counter_ = 0;
            }
        }

        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, loop_period_us);
    }
}

bool CommandLineInterface::process_serial_data()
{
    handleBluetoothCommand();
    drainConsole();

    if (deps_.battery != nullptr && deps_.sensor_mode != SensorMode::TOF)
        deps_.battery->update();

    bool processed = false;
    if (deps_.bluetooth != nullptr)
    {
        char bluetooth_line[LINE_BUFFER_SIZE] = {};
        while (deps_.bluetooth->readLine(bluetooth_line, sizeof(bluetooth_line)))
        {
            process_line(bluetooth_line);
            prompt();
            processed = true;
        }
    }

    while (true)
    {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c < 0)
            break;

        char ch = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (ch == '\r' || ch == '\n')
        {
            print("\n");
            process_input_line();
            processed = true;
            continue;
        }

        if (ch == kBackspace || ch == 127)
        {
            handle_backspace();
            continue;
        }

        if (std::isprint(static_cast<unsigned char>(ch)))
            add_to_buffer(ch);
    }

    return processed;
}

void CommandLineInterface::handle_backspace()
{
    if (line_index_ <= 0)
        return;

    --line_index_;
    line_buffer_[line_index_] = '\0';
}

void CommandLineInterface::add_to_buffer(char c)
{
    if (line_index_ >= LINE_BUFFER_SIZE - 1)
        return;

    line_buffer_[line_index_++] = c;
    line_buffer_[line_index_]   = '\0';
}

int CommandLineInterface::tokenise(Args& args, char* line)
{
    args.argc = 0;
    for (char* token = std::strtok(line, " ,="); token != nullptr;
         token       = std::strtok(nullptr, " ,="))
    {
        args.argv[args.argc++] = token;
        if (args.argc >= CLI_MAX_ARGC)
            break;
    }
    return args.argc;
}

void CommandLineInterface::process_input_line()
{
    process_line(line_buffer_);
    clear_input_buffer();
    prompt();
}

void CommandLineInterface::process_line(char* line)
{
    Args args;
    if (tokenise(args, line) > 0)
        execute_command(args);
}

void CommandLineInterface::execute_command(Args& args)
{
    if (std::strlen(args.argv[0]) == 1)
        run_short_cmd(args);
    else
        run_long_cmd(args);
}

void CommandLineInterface::run_short_cmd(const Args& args)
{
    char c = args.argv[0][0];
    switch (c)
    {
        case '?':
            help();
            break;
        case 'X':
            stop();
            break;
        case 'W':
        case 'C':
        case 'D':
            printMazeView(c);
            break;
        case 'B':
            run_function(10);
            break;
        case 'G':
        case 'H':
        case 'J':
            // Gesture-shortcut letters (G=front, H=right, J=left). Only act
            // on them while COMP is armed — outside COMP they're harmless
            // no-ops to avoid surprising operators using diagnostic letters.
            if (comp_armed_)
            {
                comp_pending_trigger_ = (c == 'G')   ? StartTrigger::FRONT_WAVE
                                        : (c == 'H') ? StartTrigger::RIGHT_WAVE
                                                     : StartTrigger::LEFT_WAVE;
            }
            else
            {
                printFormat("(%c is a COMP-mode gesture shortcut — type COMP first.)\n", c);
            }
            break;
        case 'S':
            dumpSensorsOneShot();
            break;
        case 'E':
            printEncoderSnapshot();
            break;
        case 'Q':
            printEncoderSnapshot();
            break;
        case 'F':
        {
            int     function = -1;
            uint8_t digits   = args.argc >= 2 ? read_integer(args.argv[1], function) : 0;
            if (digits > 0)
            {
                run_function(function);
                last_function_ = function;
            }
            else
            {
                printFormat("F expects a function number.\n");
            }
            break;
        }
        default:
            printFormat("UNKNOWN COMMAND: ");
            printArgs(args);
            break;
    }
}

void CommandLineInterface::run_long_cmd(const Args& args)
{
    if (std::strcmp(args.argv[0], "HELP") == 0)
    {
        if (args.argc >= 2 && std::strcmp(args.argv[1], "DEBUG") == 0)
            help_debug();
        else
            help();
        return;
    }
    if (std::strcmp(args.argv[0], "SEARCH") == 0)
    {
        handle_search_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "STAGE") == 0)
    {
        handle_stage_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "SEARCHGOAL") == 0)
    {
        run_competition_stage(1, /*wait_for_start=*/true);
        return;
    }
    if (std::strcmp(args.argv[0], "RETURNSTART") == 0)
    {
        run_competition_stage(2, /*wait_for_start=*/true);
        return;
    }
    if (std::strcmp(args.argv[0], "FASTGOAL") == 0)
    {
        run_competition_stage(3, /*wait_for_start=*/true);
        return;
    }
    if (std::strcmp(args.argv[0], "FASTSTART") == 0)
    {
        run_competition_stage(4, /*wait_for_start=*/true);
        return;
    }
    if (std::strcmp(args.argv[0], "DIAGGOAL") == 0)
    {
        run_competition_stage(5, /*wait_for_start=*/true);
        return;
    }
    if (std::strcmp(args.argv[0], "COMP") == 0)
    {
        handle_comp_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "CHEESE") == 0)
    {
        runCheeseHuntMode();
        return;
    }
    if (std::strcmp(args.argv[0], "SIMRUN") == 0)
    {
        if (!run_competition_stage(1, /*wait_for_start=*/true))
        {
            printFormat("SIMRUN failed at SEARCHGOAL.\n");
            return;
        }
        if (!run_competition_stage(2, /*wait_for_start=*/false))
        {
            printFormat("SIMRUN failed at RETURNSTART.\n");
            return;
        }
        if (!run_competition_stage(5, /*wait_for_start=*/false))
        {
            printFormat("SIMRUN failed at DIAGGOAL.\n");
            return;
        }
        printFormat("SIMRUN done.\n");
        return;
    }
    if (std::strcmp(args.argv[0], "STYLE") == 0)
    {
        handle_style_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "PATH") == 0)
    {
        handle_path_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "CENTER") == 0 || std::strcmp(args.argv[0], "STARTCENTER") == 0)
    {
        handle_center_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "LINE") == 0)
    {
        handle_line_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "HALT") == 0)
    {
        stop();
        return;
    }
    if (std::strcmp(args.argv[0], "RESET") == 0)
    {
        reset();
        return;
    }

    printFormat("UNKNOWN COMMAND: ");
    printArgs(args);
}

void CommandLineInterface::handle_search_command(const Args& args)
{
    if (!needsTof("SEARCH"))
        return;
    if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
    {
        printFormat("Maze Mouse not initialized.\n");
        return;
    }

    int x = 7;
    int y = 7;
    if (args.argc >= 2)
        read_integer(args.argv[1], x);
    if (args.argc >= 3)
        read_integer(args.argv[2], y);

    if (!startWithGesture(true))
        return;

    deps_.mouse->set_hand_start(true);
    printFormat("Search to %d,%d\n", x, y);
    std::vector<std::array<int, 2>> goals = {{x, y}};
    const bool                      ok    = deps_.mouse->search_to(goals);
    printFormat(ok ? "Search done.\n" : "Search failed.\n");
}

void CommandLineInterface::handle_stage_command(const Args& args)
{
    int stage = 0;
    if (args.argc < 2 || read_integer(args.argv[1], stage) == 0)
    {
        printFormat("STAGE expects 1..5.\n");
        return;
    }

    run_competition_stage(stage, /*wait_for_start=*/true);
}

void CommandLineInterface::handle_comp_command(const Args& args)
{
    (void)args;
    runCompetitionMode();
}

void CommandLineInterface::runCompetitionMode()
{
    if (!needsTof("COMP"))
        return;
    if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
    {
        printFormat("COMP: Mouse not initialized.\n");
        return;
    }

    printFormat("COMP mode: front=Stage1+2, right=Stage3, left=Stage5. BOOTSEL aborts.\n");
    if (deps_.bluetooth != nullptr)
        printFormat(
            "CLI commands (W/C/D/S/E/B/F<n>/...) remain available over USB/BT while armed.\n");
    else
        printFormat("USB CLI commands (W/C/D/S/E/B/F<n>/...) remain available while armed.\n");

    // Run forever until the operator cancels with HALT/BOOTSEL while armed.
    // Each iteration: arm, wait for a gesture, dispatch, return to arm.
    while (true)
    {
        halted_               = false;
        comp_pending_trigger_ = StartTrigger::NONE;
        comp_armed_           = true;
        stage_led::setArmed();
        printFormat("COMP armed. Wave front=stage1+2, right=stage3, left=stage5.\n");
        prompt(); // give the operator a CLI prompt so they can type diagnostics

        // The service callback runs the full CLI input pipeline (so W/C/D/E
        // and any other commands work) and surfaces gesture shortcuts and
        // BT START via comp_pending_trigger_. waitForCompetitionGesture
        // also polls BOOTSEL directly so a press while armed cancels.
        StartTrigger trigger = waitForCompetitionGesture(
            deps_.bluetooth, deps_.front_tof, deps_.right_tof, deps_.left_tof, /*low_mm=*/80,
            /*high_mm=*/110, &CommandLineInterface::compServiceThunk, this);
        comp_armed_ = false;

        if (trigger == StartTrigger::CANCELLED)
        {
            stage_led::setIdle();
            printFormat("COMP exit.\n");
            return;
        }

        // Snapshot the maze before the run starts. If BOOTSEL aborts mid-run
        // the mouse's pose is unreliable, so any walls it logged this run
        // can't be trusted — restore() rolls back to the pre-run state and
        // preserves wall data from earlier successful runs. Stages 3 and 5
        // do not write walls, so the restore is a no-op for them, but we
        // take the snapshot uniformly so the abort path is identical.
        MazeSnapshot pre_run_snapshot;
        if (deps_.maze != nullptr)
            pre_run_snapshot = deps_.maze->snapshot();

        bool ok = false;
        switch (trigger)
        {
            case StartTrigger::FRONT_WAVE:
            case StartTrigger::SERIAL_G:
            case StartTrigger::BT_START:
                printFormat("COMP gesture: FRONT -> Stage 1+2.\n");
                deps_.mouse->set_hand_start(true);
                ok = run_competition_stage(1, /*wait_for_start=*/false);
                if (ok)
                {
                    stage_led::setGoalReached();
                    sleep_ms(1000);
                    ok = run_competition_stage(2, /*wait_for_start=*/false);
                }
                if (ok)
                {
                    // After Stage 2 the mouse is parked at the start cell
                    // centre, facing the direction it came from. Spin 180 so
                    // it faces the canonical start orientation, then reverse
                    // by HALF_CELL - WHEEL_RADIUS so the rear of the wheel
                    // ends flush against the back wall — same physical pose
                    // a hand-placed start would have. Hold for ~2.5 s so the
                    // operator can readjust before the next gesture.
                    constexpr float kBackToWallMm   = HALF_CELL_MM - (WHEEL_DIAMETER_MM / 2.0f);
                    constexpr float kBackSpeedMmps  = 80.0f;
                    constexpr float kBackAccelMmps2 = 1000.0f;
                    printFormat("COMP: 180-turn + reverse %.1f mm to start wall.\n", kBackToWallMm);
                    stage_led::setGoalReached();
                    deps_.mouse->turn_IP180();
                    deps_.mouse->move_physical(-kBackToWallMm, kBackSpeedMmps, kBackAccelMmps2);
                    sleep_ms(2500);
                }
                break;

            case StartTrigger::RIGHT_WAVE:
            case StartTrigger::SERIAL_H:
                printFormat("COMP gesture: RIGHT -> Stage 3.\n");
                ok = run_competition_stage(3, /*wait_for_start=*/false);
                break;

            case StartTrigger::LEFT_WAVE:
            case StartTrigger::SERIAL_J:
                printFormat("COMP gesture: LEFT -> Stage 5.\n");
                ok = run_competition_stage(5, /*wait_for_start=*/false);
                break;

            default:
                ok = false;
                break;
        }

        if (!ok)
        {
            // Roll back wall data and pose so the next run starts from a
            // clean, trusted state. The operator picks up the mouse, places
            // it back at start, and re-gestures.
            if (deps_.maze != nullptr)
                deps_.maze->restore(pre_run_snapshot);
            if (deps_.maze_mouse != nullptr)
                deps_.maze_mouse->reset(deps_.start_cell, "n", deps_.goal_cells);
            if (deps_.mouse != nullptr)
                deps_.mouse->setUp(deps_.start_cell, deps_.goal_cells);
            printFormat("COMP run aborted: wall data rolled back, pose reset to start.\n");
            stage_led::flashAborted();

            // Enter PAUSED state. COMP does NOT auto-arm — the operator must
            // press BOOTSEL a second time to resume. This gives time to
            // inspect, reposition, and prep without the gesture detector
            // firing on incidental hand motion. Bluetooth HALT or serial
            // 'X'/'x' exit COMP entirely from the paused state.
            stage_led::setPaused();
            printFormat("COMP paused. BOOTSEL again to resume; HALT or X to exit.\n");

            // Step 1: wait for the BOOTSEL press that triggered the abort to
            // be released. Without this, the next loop iteration sees the
            // line still low and would treat it as the resume press.
            while (abortRequested())
                sleep_ms(20);
            sleep_ms(150); // settle / debounce

            // Step 2: wait for a fresh press, while still honoring HALT/X.
            bool exit_comp = false;
            while (true)
            {
                if (abortRequested())
                    break;

                Bluetooth* bt = deps_.bluetooth;
                if (bt != nullptr)
                {
                    Log::drainBluetooth();
                    bt->drain();
                    if (bt->hasCommand() && bt->command() == Bluetooth::Command::HALT)
                    {
                        exit_comp = true;
                        break;
                    }
                }
                int c = getchar_timeout_us(0);
                if (c == 'X' || c == 'x')
                {
                    exit_comp = true;
                    break;
                }

                sleep_ms(20);
            }

            if (exit_comp)
            {
                stage_led::setIdle();
                printFormat("COMP exit (paused).\n");
                return;
            }

            // Step 3: wait for the resume press to be released so the next
            // call to waitForCompetitionGesture doesn't see a held BOOTSEL
            // and immediately CANCEL into another exit.
            while (abortRequested())
                sleep_ms(20);
            sleep_ms(150);

            printFormat("COMP resume.\n");
            // Falls through to the top of the while(true): setArmed() and
            // wait for the next gesture against the rolled-back maze.
        }
    }
}

void CommandLineInterface::runCheeseHuntMode()
{
    if (!needsTof("CHEESE"))
        return;
    if (deps_.mouse == nullptr || deps_.maze == nullptr || deps_.maze_mouse == nullptr ||
        deps_.beacon_ir == nullptr)
    {
        printFormat("Cheese Hunt: missing mouse, maze, or beacon IR dependency.\n");
        return;
    }

    printFormat("Cheese Hunt: front wave starts beacon search. Bluetooth is disabled.\n");
    stage_led::setCheeseHunting();

    StartTrigger trigger =
        waitForStartGesture(/*bt=*/nullptr, deps_.front_tof, /*low_mm=*/80, /*high_mm=*/110);
    if (trigger == StartTrigger::CANCELLED)
    {
        stage_led::setIdle();
        printFormat("Cheese Hunt: cancelled before start.\n");
        return;
    }

    deps_.maze->reset();
    deps_.maze_mouse->reset(deps_.start_cell, "n", deps_.goal_cells);
    deps_.mouse->setUp(deps_.start_cell, deps_.goal_cells);
    deps_.mouse->set_hand_start(true);
    deps_.beacon_ir->reset();

    printFormat("Cheese Hunt: hunting from start=%s.\n", coordinateText(deps_.start_cell).c_str());
    const bool found = deps_.mouse->cheese_hunt(deps_.start_cell, deps_.beacon_ir);

    if (found || deps_.beacon_ir->beaconOff())
    {
        stage_led::setBeaconFound();
        printFormat("Cheese Hunt: beacon deactivated; returned to start if reachable.\n");
    }
    else
    {
        stage_led::setCheeseHunting();
        printFormat("Cheese Hunt: beacon not found in reachable explored maze.\n");
    }

    if (deps_.motion != nullptr)
    {
        deps_.motion->stop();
        deps_.motion->disable_drive();
    }
}

void CommandLineInterface::handle_style_command(const Args& args)
{
    if (deps_.mouse == nullptr)
    {
        printFormat("Mouse not initialized.\n");
        return;
    }

    if (args.argc < 2)
    {
        printFormat("STYLE: %s\n", movementStyleName(deps_.mouse->movementStyle()));
        return;
    }

    if (std::strcmp(args.argv[1], "SMOOTH") == 0)
    {
        deps_.mouse->setMovementStyle(Mouse::MovementStyle::Smooth);
        printFormat("STYLE: SMOOTH\n");
        return;
    }

    if (std::strcmp(args.argv[1], "STATIONARY") == 0 || std::strcmp(args.argv[1], "STILL") == 0)
    {
        deps_.mouse->setMovementStyle(Mouse::MovementStyle::Stationary);
        printFormat("STYLE: STATIONARY\n");
        return;
    }

    printFormat("STYLE expects STATIONARY or SMOOTH.\n");
}

void CommandLineInterface::append_path_forward_cells(std::vector<PathSegment>& segments, int cells)
{
    const float distance_mm = cells * CELL_SIZE_MM;
    if (!segments.empty() && segments.back().type == PathSegmentType::Forward)
    {
        segments.back().value += distance_mm;
        return;
    }
    segments.push_back({PathSegmentType::Forward, distance_mm});
}

void CommandLineInterface::append_path_turn(std::vector<PathSegment>& segments, float degrees)
{
    segments.push_back({PathSegmentType::Turn, degrees});
}

bool CommandLineInterface::parse_path_chunk(const char* chunk, std::vector<PathSegment>& segments)
{
    if (chunk == nullptr)
        return false;

    const char* p = chunk;
    while (*p != '\0')
    {
        if (*p == '#')
        {
            ++p;
            continue;
        }

        const char token = static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
        ++p;

        if (token == 'F')
        {
            int  cells      = 0;
            bool has_digits = false;
            while (std::isdigit(static_cast<unsigned char>(*p)))
            {
                has_digits = true;
                cells      = cells * 10 + (*p - '0');
                ++p;
            }
            if (has_digits && cells <= 0)
            {
                printFormat("PATH forward count must be positive.\n");
                return false;
            }
            append_path_forward_cells(segments, cells == 0 ? 1 : cells);
            continue;
        }

        if (token == 'L' || token == 'R' || token == 'B')
        {
            if (std::isdigit(static_cast<unsigned char>(*p)))
            {
                printFormat("PATH only supports L/R/B as turns, not %c%c...\n", token, *p);
                return false;
            }
            if (token == 'L')
                append_path_turn(segments, 90.0f);
            else if (token == 'R')
                append_path_turn(segments, -90.0f);
            else
                append_path_turn(segments, 180.0f);
            continue;
        }

        printFormat("Invalid PATH token near '%c'.\n", token);
        return false;
    }

    return true;
}

bool CommandLineInterface::parse_path_sequence(const Args& args, int first_param_index,
                                               std::vector<PathSegment>& segments)
{
    segments.clear();
    for (int i = 1; i < first_param_index; ++i)
    {
        if (!parse_path_chunk(args.argv[i], segments))
        {
            segments.clear();
            return false;
        }
    }

    if (segments.empty())
    {
        printFormat("PATH expects a sequence, e.g. PATH FFFFLFFFFLFFF\n");
        return false;
    }
    return true;
}

bool CommandLineInterface::run_path_segments(std::vector<PathSegment>& segments, float speed_mmps,
                                             float accel_mmps2, float omega_degps,
                                             float alpha_degps2, bool smooth_turns, bool raw_mode)
{
    if (deps_.mouse == nullptr || deps_.motion == nullptr)
    {
        printFormat("PATH requires Motion/Mouse in ToF CLI mode.\n");
        return false;
    }

    if (smooth_turns)
    {
        for (PathSegment& segment : segments)
        {
            if (segment.type == PathSegmentType::Turn &&
                std::fabs(std::fabs(segment.value) - 90.0f) < 0.125f)
            {
                segment.type = PathSegmentType::SmoothTurn;
            }
        }
    }

    deps_.motion->begin_motion_sequence();
    halted_ = false;

    float expected_yaw_deg = 0.0f;
    float total_forward_mm = 0.0f;

    printFormat("PATH segments=%lu speed=%.1f accel=%.1f omega=%.1f alpha=%.1f smooth=%s raw=%s\n",
                static_cast<unsigned long>(segments.size()), static_cast<double>(speed_mmps),
                static_cast<double>(accel_mmps2), static_cast<double>(omega_degps),
                static_cast<double>(alpha_degps2), smooth_turns ? "true" : "false",
                raw_mode ? "true" : "false");

    if (!raw_mode)
    {
        deps_.motion->set_heading_hold(0.0f);
        deps_.motion->move(START_CENTER_DISTANCE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f,
                           ROBOT_BASE_ACCEL_MMPS2);
        if (!wait_path_segment_motion())
        {
            deps_.motion->end_motion_sequence();
            return false;
        }
        deps_.motion->set_position(HALF_CELL_MM);
        printFormat("PATH hand_start: advance=%.1f mm pos=%.1f mm\n",
                    static_cast<double>(START_CENTER_DISTANCE_MM),
                    static_cast<double>(HALF_CELL_MM));
    }

    for (size_t i = 0; i < segments.size(); ++i)
    {
        const PathSegment& segment        = segments[i];
        bool               ok             = true;
        const float        yaw_before_deg = deps_.motion->yaw_deg();

        if (segment.type == PathSegmentType::Forward)
        {
            printFormat("PATH %lu/%lu: F %.1f mm\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            if (!raw_mode)
                deps_.motion->set_heading_hold(normalizeYawDelta(expected_yaw_deg));
            deps_.motion->start_move(segment.value, speed_mmps, 0.0f, accel_mmps2);
            ok = wait_path_segment_motion();
            total_forward_mm += segment.value;
        }
        else if (segment.type == PathSegmentType::SmoothTurn)
        {
            if (!raw_mode)
                deps_.motion->clear_heading_hold();
            const int                   turn_id = segment.value >= 0.0f ? SS90L : SS90R;
            const SmoothTurnParameters& params  = SMOOTH_TURN_PARAMS[turn_id];
            printFormat(
                "PATH %lu/%lu: SMOOTH %.1f deg speed=%.1f omega=%.1f\n",
                static_cast<unsigned long>(i + 1), static_cast<unsigned long>(segments.size()),
                static_cast<double>(params.angle_deg), static_cast<double>(params.speed_mmps),
                static_cast<double>(params.omega_degps));
            deps_.motion->turn_smooth(turn_id);
            ok = wait_path_segment_motion();
            expected_yaw_deg += params.angle_deg;
        }
        else
        {
            if (!raw_mode)
                deps_.motion->clear_heading_hold();
            printFormat("PATH %lu/%lu: TURN %.1f deg\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            deps_.motion->spin_turn(segment.value, std::fabs(omega_degps), std::fabs(alpha_degps2));
            ok = wait_path_segment_motion();
            expected_yaw_deg += segment.value;
        }

        const float yaw_after_deg        = deps_.motion->yaw_deg();
        const float actual_delta_deg     = normalizeYawDelta(yaw_after_deg - yaw_before_deg);
        const float segment_error_deg    = normalizeYawDelta(expected_yaw_deg - yaw_after_deg);
        const float expected_yaw_wrapped = normalizeYawDelta(expected_yaw_deg);
        if (!raw_mode && segment.type == PathSegmentType::Forward)
        {
            printFormat(
                "PATH %lu/%lu result: yaw_before=%.2f yaw_after=%.2f delta=%+.2f "
                "expected_yaw=%.2f yaw_error=%+.2f hold_err=%+.2f hold_rate=%+.2f\n",
                static_cast<unsigned long>(i + 1), static_cast<unsigned long>(segments.size()),
                static_cast<double>(yaw_before_deg), static_cast<double>(yaw_after_deg),
                static_cast<double>(actual_delta_deg), static_cast<double>(expected_yaw_wrapped),
                static_cast<double>(segment_error_deg),
                static_cast<double>(deps_.motion->headingHoldErrorDeg()),
                static_cast<double>(deps_.motion->headingHoldAdjustmentDegps()));
        }
        else if (segment.type == PathSegmentType::Turn)
        {
            printFormat(
                "PATH %lu/%lu result: cmd=%+.2f yaw_before=%.2f yaw_after=%.2f delta=%+.2f "
                "expected_yaw=%.2f yaw_error=%+.2f\n",
                static_cast<unsigned long>(i + 1), static_cast<unsigned long>(segments.size()),
                static_cast<double>(segment.value), static_cast<double>(yaw_before_deg),
                static_cast<double>(yaw_after_deg), static_cast<double>(actual_delta_deg),
                static_cast<double>(expected_yaw_wrapped), static_cast<double>(segment_error_deg));
        }
        else
        {
            printFormat(
                "PATH %lu/%lu result: yaw_before=%.2f yaw_after=%.2f delta=%+.2f "
                "expected_yaw=%.2f yaw_error=%+.2f\n",
                static_cast<unsigned long>(i + 1), static_cast<unsigned long>(segments.size()),
                static_cast<double>(yaw_before_deg), static_cast<double>(yaw_after_deg),
                static_cast<double>(actual_delta_deg), static_cast<double>(expected_yaw_wrapped),
                static_cast<double>(segment_error_deg));
        }

        if (!ok || halted_)
        {
            printFormat("PATH stopped at segment %lu.\n", static_cast<unsigned long>(i + 1));
            deps_.motion->end_motion_sequence();
            return false;
        }
    }

    deps_.motion->stop();
    const float final_yaw   = deps_.motion->yaw_deg();
    const float final_error = normalizeYawDelta(expected_yaw_deg - final_yaw);
    printFormat("PATH done: segments=%lu forward=%.1f mm (%.2f cells) expected_yaw=%.2f "
                "final_yaw=%.2f error=%+.2f deg\n",
                static_cast<unsigned long>(segments.size()), static_cast<double>(total_forward_mm),
                static_cast<double>(total_forward_mm / CELL_SIZE_MM),
                static_cast<double>(expected_yaw_deg), static_cast<double>(final_yaw),
                static_cast<double>(final_error));
    deps_.motion->end_motion_sequence();
    return true;
}

bool CommandLineInterface::wait_path_segment_motion()
{
    if (deps_.motion == nullptr)
        return false;

    while (!deps_.motion->move_finished() || !deps_.motion->turn_finished())
    {
        if (deps_.mouse != nullptr)
            deps_.mouse->serviceSensors();
        pollHaltOnly();
        if (halted_)
        {
            deps_.motion->emergency_stop();
            return false;
        }
        sleep_ms(2);
    }
    return true;
}

void CommandLineInterface::handle_path_command(const Args& args)
{
    if (!needsTof("PATH"))
        return;

    bool smooth_turns = false;
    bool raw_mode     = false;
    int  arg_limit    = args.argc;
    while (arg_limit > 2 && parsePathOption(args.argv[arg_limit - 1], smooth_turns, raw_mode))
        --arg_limit;

    int first_param_index = arg_limit;
    for (int i = 1; i < arg_limit; ++i)
    {
        if (startsNumericArg(args.argv[i]))
        {
            first_param_index = i;
            break;
        }
    }

    std::vector<PathSegment> segments;
    if (!parse_path_sequence(args, first_param_index, segments))
        return;

    float speed = PATH_DEFAULT_SPEED_MMPS;
    float accel = PATH_DEFAULT_ACCEL_MMPS2;
    float omega = PATH_DEFAULT_OMEGA_DEGPS;
    float alpha = PATH_DEFAULT_ALPHA_DEGPS2;

    int param = first_param_index;
    if (param < arg_limit &&
        !parseFloatArg(args, param++, 1.0f, CLI_PATH_NO_SPEED_LIMIT_MMPS, speed))
    {
        printFormat("PATH speed must be a positive mm/s value.\n");
        return;
    }
    if (param < arg_limit && !parseFloatArg(args, param++, 1.0f, CLI_PATH_MAX_ACCEL_MMPS2, accel))
    {
        printFormat("PATH accel must be 1..%.0f mm/s^2.\n",
                    static_cast<double>(CLI_PATH_MAX_ACCEL_MMPS2));
        return;
    }
    if (param < arg_limit && !parseFloatArg(args, param++, 1.0f, CLI_PATH_MAX_OMEGA_DEGPS, omega))
    {
        printFormat("PATH omega must be 1..%.0f deg/s.\n",
                    static_cast<double>(CLI_PATH_MAX_OMEGA_DEGPS));
        return;
    }
    if (param < arg_limit && !parseFloatArg(args, param++, 1.0f, CLI_PATH_MAX_ALPHA_DEGPS2, alpha))
    {
        printFormat("PATH alpha must be 1..%.0f deg/s^2.\n",
                    static_cast<double>(CLI_PATH_MAX_ALPHA_DEGPS2));
        return;
    }
    if (param < arg_limit)
    {
        printFormat("PATH usage: PATH <sequence> [speed_mmps] [accel_mmps2] [omega_degps] "
                    "[alpha_degps2] [SMOOTH] [RAW]\n");
        return;
    }

    run_path_segments(segments, speed, accel, omega, alpha, smooth_turns, raw_mode);
}

void CommandLineInterface::handle_center_command(const Args& args)
{
    if (!needsTof("CENTER"))
        return;

    float speed = PATH_DEFAULT_SPEED_MMPS;
    float accel = PATH_DEFAULT_ACCEL_MMPS2;

    if (args.argc > 1 && !parseFloatArg(args, 1, 1.0f, CLI_PATH_NO_SPEED_LIMIT_MMPS, speed))
    {
        printFormat("CENTER speed must be a positive mm/s value.\n");
        return;
    }
    if (args.argc > 2 && !parseFloatArg(args, 2, 1.0f, CLI_PATH_MAX_ACCEL_MMPS2, accel))
    {
        printFormat("CENTER accel must be 1..%.0f mm/s^2.\n",
                    static_cast<double>(CLI_PATH_MAX_ACCEL_MMPS2));
        return;
    }
    if (args.argc > 3)
    {
        printFormat("CENTER usage: CENTER [speed_mmps] [accel_mmps2]\n");
        return;
    }

    printFormat("CENTER distance: %.1f mm\n", static_cast<double>(START_CENTER_DISTANCE_MM));
    if (deps_.motion != nullptr)
        deps_.motion->reset_drive_system();
    if (deps_.mouse == nullptr ||
        !deps_.mouse->move_physical(START_CENTER_DISTANCE_MM, speed, accel))
        printFormat("CENTER failed.\n");
}

void CommandLineInterface::handle_line_command(const Args& args)
{
    if (deps_.sensor_mode != SensorMode::LINE_SENSOR || deps_.line_follower == nullptr)
    {
        printFormat("LINE requires LineSensor mode. Reboot Normal CLI and select 'L'.\n");
        return;
    }

    if (args.argc < 2 || std::strcmp(args.argv[1], "STATUS") == 0)
    {
        printLineSnapshot();
        return;
    }

    if (std::strcmp(args.argv[1], "TUNE") == 0)
    {
        printLineTuning();
        return;
    }

    if (std::strcmp(args.argv[1], "DEFAULTS") == 0)
    {
        deps_.line_follower->resetRuntimeTuning();
        printFormat("LINE DEFAULTS restored from config/tuning.h\n");
        printLineTuning();
        return;
    }

    if (std::strcmp(args.argv[1], "SET") == 0)
    {
        if (args.argc < 4)
        {
            printFormat("LINE SET usage: LINE SET <name> <value>\n");
            return;
        }

        float value = 0.0f;
        if (!parseFloatArg(args, 3, -10000.0f, 10000.0f, value))
        {
            printFormat("LINE SET value must be numeric.\n");
            return;
        }

        if (!deps_.line_follower->setRuntimeTuningValue(args.argv[2], value))
        {
            printFormat("LINE SET invalid name/range. Editable: LINE_KP_BASE_DEGPS_PER_SLOT, "
                        "LINE_KD_BASE_DEG_PER_SLOT, LINE_TARGET_SPEED_MMPS, "
                        "LINE_MAX_SPEED_MMPS, LINE_MIN_SPEED_MMPS.\n");
            return;
        }

        printFormat("LINE SET OK %s=%.6f\n", args.argv[2], static_cast<double>(value));
        printLineTuning();
        return;
    }

    if (std::strcmp(args.argv[1], "START") == 0)
    {
        // Arm-then-go pattern, mirroring COMP's BOOTSEL handshake. The
        // banner prints the active config so a tail of the serial log
        // unambiguously shows which tuning produced any given run.
        printLineStartBanner();
        if (!waitForLineStartButton())
        {
            printFormat("LINE START cancelled.\n");
            return;
        }
        deps_.line_follower->startFollowing();
        printLineTelemetryHeader();
        const LineFollower::RuntimeTuning& tune = deps_.line_follower->runtimeTuning();
        printFormat("LINE START -> FOLLOWING (target=%.0f mm/s, max=%.0f, min=%.0f)\n",
                    static_cast<double>(tune.target_speed_mmps),
                    static_cast<double>(tune.max_speed_mmps),
                    static_cast<double>(tune.min_speed_mmps));
        return;
    }

    if (std::strcmp(args.argv[1], "STOP") == 0)
    {
        printLineRunSummary();
        deps_.line_follower->stop();
        printFormat("LINE STOP\n");
        return;
    }

    if (std::strcmp(args.argv[1], "LEFT") == 0 || std::strcmp(args.argv[1], "L") == 0)
    {
        deps_.line_follower->turnLeft90();
        printFormat("LINE LEFT\n");
        return;
    }

    if (std::strcmp(args.argv[1], "RIGHT") == 0 || std::strcmp(args.argv[1], "R") == 0)
    {
        deps_.line_follower->turnRight90();
        printFormat("LINE RIGHT\n");
        return;
    }

    if (std::strcmp(args.argv[1], "ROUTE") == 0)
    {
        if (args.argc < 3)
        {
            printFormat("LINE ROUTE usage: LINE ROUTE <LFR...|CLEAR|STATUS>\n");
            return;
        }

        if (std::strcmp(args.argv[2], "STATUS") == 0)
        {
            printLineSnapshot();
            return;
        }

        if (std::strcmp(args.argv[2], "CLEAR") == 0)
        {
            deps_.line_follower->clearRoute();
            printFormat("LINE ROUTE cleared\n");
            return;
        }

        if (!deps_.line_follower->setRoute(args.argv[2]))
        {
            printFormat("LINE ROUTE invalid. Use only L/F/R (optional spaces/commas), max 64.\n");
            return;
        }

        printFormat("LINE ROUTE set: %s\n", deps_.line_follower->route());
        return;
    }

    printFormat("LINE usage: LINE [STATUS|TUNE|SET|DEFAULTS|START|STOP|LEFT|RIGHT|ROUTE]\n");
}

bool CommandLineInterface::run_competition_stage(int stage, bool wait_for_start)
{
    if (!needsTof("STAGE"))
        return false;
    if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
    {
        printFormat("Maze Mouse not initialized.\n");
        return false;
    }

    if (stage < 1 || stage > 5)
    {
        printFormat("STAGE expects 1..5.\n");
        return false;
    }

    const bool preserve_yaw = (stage == 2 || stage == 4);
    if (wait_for_start && !startWithGesture(true, preserve_yaw))
        return false;

    if (stage == 1 || stage == 3 || stage == 5)
    {
        if (!wait_for_start && deps_.sensor_mode == SensorMode::TOF && deps_.motion != nullptr)
            deps_.motion->reset_drive_system();
        deps_.maze_mouse->reset(deps_.start_cell, "n", deps_.goal_cells);
        deps_.mouse->set_heading("n");
    }

    Cell*       current_cell = deps_.maze_mouse->currentCell();
    std::string current_text = current_cell != nullptr
                                   ? "(" + std::to_string(current_cell->x()) + "," +
                                         std::to_string(current_cell->y()) + ")"
                                   : "(none)";
    std::string heading_text = deps_.maze_mouse->currentDirection();
    std::string start_text   = coordinateText(deps_.start_cell);
    std::string goal_text    = goalsText(deps_.goal_cells);
    printFormat("STAGE config: current=%s heading=%s start=%s goals=%s mask=%s\n",
                current_text.c_str(), heading_text.c_str(), start_text.c_str(), goal_text.c_str(),
                mazeMaskName(deps_.maze_mouse->mazeMask()));

    stage_led::setStage(stage);

    switch (stage)
    {
        case 1:
            printFormat("Stage 1: iterative A* search to goal.\n");
            deps_.mouse->setPhaseColor('y');
            if (wait_for_start)
                deps_.mouse->set_hand_start(true);
            return deps_.mouse->search_to(deps_.goal_cells);

        case 2:
        {
            printFormat("Stage 2: iterative A* return to start.\n");
            deps_.mouse->setPhaseColor('c');
            std::vector<std::array<int, 2>> goals = {deps_.start_cell};
            return deps_.mouse->search_to(goals);
        }

        case 3:
        {
            printFormat("Stage 3: explored-only cardinal fast run to goal.\n");
            deps_.mouse->setPhaseColor('g');
            const float prev_cruise = deps_.mouse->cruiseSpeed();
            deps_.mouse->setCruiseSpeed(ROBOT_MAX_FAST_SPEED_MMPS);
            const bool ok = PathUtils::traverseExploredPath(
                deps_.mouse, deps_.maze_mouse, deps_.goal_cells, /*start_from_back_wall=*/true);
            deps_.mouse->setCruiseSpeed(prev_cruise);
            return ok;
        }

        case 4:
        {
            printFormat("Stage 4: explored-only cardinal fast return to start.\n");
            deps_.mouse->setPhaseColor('c');
            std::vector<std::array<int, 2>> goals       = {deps_.start_cell};
            const float                     prev_cruise = deps_.mouse->cruiseSpeed();
            deps_.mouse->setCruiseSpeed(ROBOT_MAX_FAST_SPEED_MMPS);
            const bool ok = PathUtils::traverseExploredPath(deps_.mouse, deps_.maze_mouse, goals,
                                                            /*start_from_back_wall=*/false);
            deps_.mouse->setCruiseSpeed(prev_cruise);
            return ok;
        }

        case 5:
        {
            printFormat("Stage 5: explored-only diagonal fast run to goal.\n");
            deps_.mouse->setPhaseColor('G');
            const float prev_cruise = deps_.mouse->cruiseSpeed();
            deps_.mouse->setCruiseSpeed(ROBOT_MAX_FAST_SPEED_MMPS);
            const bool ok = PathUtils::traverseExploredDiagonalPath(deps_.mouse, deps_.maze_mouse,
                                                                    deps_.goal_cells,
                                                                    /*start_from_back_wall=*/true);
            deps_.mouse->setCruiseSpeed(prev_cruise);
            return ok;
        }
    }

    return false;
}

void CommandLineInterface::handleBluetoothCommand()
{
    Bluetooth* bt = deps_.bluetooth;
    if (bt == nullptr || !bt->hasCommand())
        return;

    Bluetooth::Command cmd = bt->command();
    switch (cmd)
    {
        case Bluetooth::Command::START:
            // While COMP is gesture-armed, BT START is the front-wave
            // shortcut equivalent. Surface it through the same channel as
            // the G/H/J letters so handle_comp_command sees it via its
            // service tick.
            if (comp_armed_)
            {
                LOG_INFO("BT START: COMP front gesture");
                comp_pending_trigger_ = StartTrigger::FRONT_WAVE;
            }
            else if (last_function_ >= 0)
            {
                LOG_INFO("BT START: running F " << last_function_);
                run_function(last_function_);
            }
            else
            {
                print("BT START: no last function. Use F n first.\n");
            }
            break;
        case Bluetooth::Command::HALT:
            LOG_INFO("BT HALT");
            stop();
            break;
        case Bluetooth::Command::RESET:
            LOG_INFO("BT RESET: resetting search state");
            reset();
            break;
        case Bluetooth::Command::BATTERY:
            run_function(10);
            break;
        default:
            break;
    }
}

void CommandLineInterface::pollHaltOnly()
{
    int c = getchar_timeout_us(0);
    if (c >= 0)
    {
        char ch = static_cast<char>(c);
        if (ch == 'X' || ch == 'x')
        {
            stop();
            return;
        }
    }

    Bluetooth* bt = deps_.bluetooth;
    if (bt != nullptr)
    {
        drainConsole();
    }
    if (bt != nullptr && bt->hasCommand())
    {
        Bluetooth::Command cmd = bt->command();
        if (cmd == Bluetooth::Command::HALT)
        {
            stop();
            return;
        }
    }

    // BOOTSEL on the Waveshare RP2040-Zero acts as the panic-abort during a
    // run. Reads the QSPI flash CS line via a brief XIP-suspend (~10 us with
    // interrupts disabled). Cheap enough at the 2 ms pollHaltOnly cadence.
    if (abortRequested())
        stop();
}

void CommandLineInterface::run_function(int cmd)
{
    halted_ = false;

    if (cmd == 0)
        return;

    switch (cmd)
    {
        case 1:
            if (!needsTof("Sensor Static Calibration"))
                break;
            if (deps_.mouse != nullptr)
                deps_.mouse->show_sensor_calibration();
            break;

        case 2:
            if (!needsTof("Search maze"))
                break;
            if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.mouse != nullptr)
                deps_.mouse->setPhaseColor('y');
            printFormat("Searching maze...\n");
            printFormat(deps_.mouse->search_maze() ? "Search done.\n" : "Search failed.\n");
            break;

        case 3:
            if (!needsTof("Follow to goal"))
                break;
            if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            printFormat("Follow to goal...\n");
            deps_.mouse->set_hand_start(true);
            printFormat(deps_.mouse->follow_to(deps_.goal_cells) ? "Follow done.\n"
                                                                 : "Follow failed.\n");
            break;

        case 4:
            if (!needsTof("Test SS90E Turn"))
                break;
            if (deps_.mouse == nullptr)
                break;
            halted_ = false;
            if (deps_.motion != nullptr)
                deps_.motion->emergency_stop();
            printFormat("Test SS90E Turn: starting immediately.\n");
            deps_.mouse->set_hand_start(true);
            printFormat(deps_.mouse->test_SS90E() ? "SS90E right done.\n" : "SS90E failed.\n");
            break;

        case 5:
            if (!needsTof("Wander"))
                break;
            if (deps_.mouse == nullptr || deps_.maze_mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            deps_.mouse->set_hand_start(true);
            printFormat(deps_.mouse->wander_to(deps_.goal_cells) ? "Wander done.\n"
                                                                 : "Wander failed.\n");
            break;

        case 6:
            if (!needsTof("Edge detect position test"))
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.mouse != nullptr)
                deps_.mouse->conf_edge_detection();
            break;

        case 7:
            if (!needsTof("Sensor Spin Calibration"))
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.mouse != nullptr)
                deps_.mouse->conf_sensor_spin_calibrate();
            break;

        case 8:
            if (!needsTof("Get Front Sensor table"))
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.mouse != nullptr)
                deps_.mouse->conf_log_front_sensor();
            break;

        case 9:
            if (!needsTof("Move forward 4 cells"))
                break;
            if (deps_.mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            printFormat(deps_.mouse->run(4.0f * CELL_SIZE_MM) ? "Run done.\n" : "Run failed.\n");
            break;

        case 10:
            if (deps_.battery == nullptr)
            {
                printFormat("Battery monitor not initialized.\n");
                break;
            }
            printFormat("Battery: %.2f Volts\n", deps_.battery->voltage());
            break;

        default:
            if (deps_.motion != nullptr)
                deps_.motion->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
            stop();
            break;
    }
}

void CommandLineInterface::dumpSensorsOneShot()
{
    if (deps_.sensor_mode == SensorMode::LINE_SENSOR)
    {
        printLineSnapshot();
        if (deps_.battery != nullptr)
            printFormat("Battery: %.2f V\n", deps_.battery->voltage());
        return;
    }

    Motion* r = deps_.motion;
    if (r != nullptr)
    {
        printFormat("ToF raw  L=%d mm F=%d mm R=%d mm\n", static_cast<int>(r->leftRawDistance()),
                    static_cast<int>(r->frontRawDistance()),
                    static_cast<int>(r->rightRawDistance()));
        printFormat("ToF filt L=%d mm F=%d mm R=%d mm  yaw=%.1f deg\n",
                    static_cast<int>(r->leftDistance()), static_cast<int>(r->frontDistance()),
                    static_cast<int>(r->rightDistance()), static_cast<double>(r->yaw_deg()));
    }
    else
    {
        printFormat("ToF: motion not initialized\n");
    }
    if (deps_.battery != nullptr)
        printFormat("Battery: %.2f V\n", deps_.battery->voltage());
    if (deps_.bluetooth != nullptr)
    {
        Log::BluetoothDiagnostics log = Log::bluetoothDiagnostics();
        Bluetooth::Diagnostics    bt  = deps_.bluetooth->diagnostics();
        printFormat("BT log queued=%u max=%u dropped=%lu truncated=%lu\n",
                    static_cast<unsigned>(log.queued_messages),
                    static_cast<unsigned>(log.max_queued_messages),
                    static_cast<unsigned long>(log.dropped_messages),
                    static_cast<unsigned long>(log.truncated_messages));
        printFormat("BT tx depth=%u max=%u dropped=%lu\n", static_cast<unsigned>(bt.ring_depth),
                    static_cast<unsigned>(bt.max_ring_depth),
                    static_cast<unsigned long>(bt.dropped_bytes));
        printFormat("BT rx lines=%u max=%u dropped_lines=%lu dropped_chars=%lu pending=%u\n",
                    static_cast<unsigned>(bt.rx_line_depth),
                    static_cast<unsigned>(bt.max_rx_line_depth),
                    static_cast<unsigned long>(bt.dropped_rx_lines),
                    static_cast<unsigned long>(bt.dropped_rx_chars),
                    static_cast<unsigned>(bt.pending_shortcut));
    }
}

void CommandLineInterface::printLineSnapshot()
{
    if (deps_.line_follower == nullptr)
    {
        printFormat("LineFollower not initialized.\n");
        return;
    }

    const LineFollower::State state = deps_.line_follower->state();
    const char*               name  = "UNKNOWN";
    switch (state)
    {
        case LineFollower::State::Idle:
            name = "IDLE";
            break;
        case LineFollower::State::FollowingLine:
            name = "FOLLOWING";
            break;
        case LineFollower::State::TurningLeft:
            name = "TURN_LEFT";
            break;
        case LineFollower::State::TurningRight:
            name = "TURN_RIGHT";
            break;
        case LineFollower::State::Stopping:
            name = "STOPPING";
            break;
    }

    const float motion_steering =
        deps_.motion != nullptr ? deps_.motion->lineSteeringAdjustmentDegps() : 0.0f;
    const bool motion_steering_valid =
        deps_.motion != nullptr ? deps_.motion->lineSteeringValid() : false;
    const float line_diff_v  = deps_.motion != nullptr ? deps_.motion->lineDifferentialVolts() : 0.0f;
    const float line_rot_v   = deps_.motion != nullptr ? deps_.motion->lineRotationVolts() : 0.0f;
    const float motor_diff_v = deps_.motion != nullptr ? deps_.motion->motorDifferentialVolts() : 0.0f;
    const std::string current_paths = linePathsText(deps_.line_follower->currentPathsMask());
    const std::string bits          = lineBitsText(deps_.line_follower->activeMask());
    const std::string route_status  = lineRouteProgressText(deps_.line_follower);
    const std::string last_decision = lineLastDecisionText(deps_.line_follower);

    printFormat("Line sensor:\n");
    printFormat("  raw=0x%02X active=0x%02X bits X8..X1 [%s] paths=%s present=%s lost=%s\n",
                deps_.line_follower->rawByte(), deps_.line_follower->activeMask(), bits.c_str(),
                current_paths.c_str(), yesNo(deps_.line_follower->linePresent()),
                yesNo(deps_.line_follower->lineLost()));
    const float velocity_now = deps_.motion != nullptr ? deps_.motion->velocity() : 0.0f;
    printFormat("  line: position=%.2f error=%.2f filtered=%.2f steer=%.1f deg/s\n",
                static_cast<double>(deps_.line_follower->linePosition()),
                static_cast<double>(deps_.line_follower->lineError()),
                static_cast<double>(deps_.line_follower->filteredLineError()),
                static_cast<double>(deps_.line_follower->steeringAdjustmentDegps()));
    printFormat("  speed: target=%.0f mm/s actual=%.0f mm/s\n",
                static_cast<double>(deps_.line_follower->targetSpeedMmps()),
                static_cast<double>(velocity_now));
    printFormat("  motion: line_steer=%.1f deg/s valid=%s state=%s\n",
                static_cast<double>(motion_steering), yesNo(motion_steering_valid), name);
    printFormat("  volts: line_diff=%+.3f line_rot=%+.3f motor_diff=%+.3f\n",
                static_cast<double>(line_diff_v), static_cast<double>(line_rot_v),
                static_cast<double>(motor_diff_v));
    printFormat("  route: %s\n", route_status.c_str());
    printFormat("  %s\n", last_decision.c_str());
}

void CommandLineInterface::printLineTuning()
{
    if (deps_.line_follower == nullptr)
        return;

    const LineFollower::RuntimeTuning& tune = deps_.line_follower->runtimeTuning();
    printFormat("LINE_TUNE\n");
    printFormat("LINE_KP_BASE_DEGPS_PER_SLOT = %.6f\n",
                static_cast<double>(tune.kp_base_degps_per_slot));
    printFormat("LINE_KD_BASE_DEG_PER_SLOT = %.6f\n",
                static_cast<double>(tune.kd_base_deg_per_slot));
    printFormat("LINE_TARGET_SPEED_MMPS = %.6f\n", static_cast<double>(tune.target_speed_mmps));
    printFormat("LINE_MAX_SPEED_MMPS = %.6f\n", static_cast<double>(tune.max_speed_mmps));
    printFormat("LINE_MIN_SPEED_MMPS = %.6f\n", static_cast<double>(tune.min_speed_mmps));
}

void CommandLineInterface::printLineStartBanner()
{
    if (deps_.line_follower == nullptr)
        return;

    // Tunable summary — captures the *parameters* of the run so a tail of
    // the serial log can be matched back to a specific tuning.h commit.
    // Mirrors the "PATH segments=N speed=X accel=Y..." entry line.
    const LineFollower::RuntimeTuning& tune = deps_.line_follower->runtimeTuning();
    printFormat("LINE config: target=%.0f mm/s max=%.0f min=%.0f omega_limit=%.0f\n",
                static_cast<double>(tune.target_speed_mmps),
                static_cast<double>(tune.max_speed_mmps),
                static_cast<double>(tune.min_speed_mmps),
                static_cast<double>(LINE_OMEGA_LIMIT_DEGPS));
    printFormat("LINE gains: Kp_base=%.1f Kd_base=%.2f ref_v=%.0f gain_floor_v=%.0f "
                "alpha=%.2f lookahead=%.0f ms\n",
                static_cast<double>(tune.kp_base_degps_per_slot),
                static_cast<double>(tune.kd_base_deg_per_slot),
                static_cast<double>(LINE_GAIN_REF_SPEED_MMPS),
                static_cast<double>(LINE_GAIN_SCHED_FLOOR_MMPS),
                static_cast<double>(LINE_ERROR_FILTER_ALPHA),
                static_cast<double>(LINE_LOOKAHEAD_TIME_S * 1000.0f));
    printFormat("LINE brake: gain=%.0f mm/s/slot lost_hold=%u ms lost_stop=%u ms recovery=%.2f\n",
                static_cast<double>(LINE_BRAKE_GAIN_MMPS_PER_SLOT),
                static_cast<unsigned>(LINE_LOST_HOLD_MS), static_cast<unsigned>(LINE_LOST_STOP_MS),
                static_cast<double>(LINE_RECOVERY_AUTHORITY));
    printFormat("LINE intersection: branch_bias=%.0f deg/s capture=%u ms lockout=%u ms\n",
                static_cast<double>(LINE_BRANCH_STEER_BIAS_DEGPS),
                static_cast<unsigned>(LINE_BRANCH_CAPTURE_MS),
                static_cast<unsigned>(LINE_INTERSECTION_LOCKOUT_MS));

    const char* route = deps_.line_follower->route();
    if (route != nullptr && route[0] != '\0')
        printFormat("LINE route: %s (%u steps)\n", route,
                    static_cast<unsigned>(std::strlen(route)));
    else
        printFormat("LINE route: <empty> (intersections default to forward)\n");

    if (deps_.battery != nullptr)
        printFormat("LINE battery: %.2f V\n", static_cast<double>(deps_.battery->voltage()));

    printFormat("LINE armed. Press BOOTSEL to start. Send 'X' / HALT to cancel.\n");
}

bool CommandLineInterface::waitForLineStartButton()
{
    halted_ = false;

    // Wait for a fresh BOOTSEL transition: must see RELEASED first (in case
    // the user is still holding from a prior press), then PRESSED. This is
    // the same pattern used by COMP-resume in runCompetitionMode.
    bool seen_released = !abortRequested();
    while (true)
    {
        // Cancel paths — match the COMP idiom so users have one mental model.
        Bluetooth* bt = deps_.bluetooth;
        if (bt != nullptr)
        {
            Log::drainBluetooth();
            bt->drain();
            if (bt->hasCommand() && bt->command() == Bluetooth::Command::HALT)
                return false;
        }
        int c = getchar_timeout_us(0);
        if (c == 'X' || c == 'x')
            return false;

        const bool pressed = abortRequested();
        if (!seen_released)
        {
            if (!pressed)
                seen_released = true;
        }
        else if (pressed)
        {
            // Debounce + wait for release so the in-loop BOOTSEL stop poll
            // doesn't immediately trigger from the same press.
            sleep_ms(20);
            while (abortRequested())
                sleep_ms(20);
            sleep_ms(100);
            return true;
        }

        sleep_ms(20);
    }
}

void CommandLineInterface::printLineTelemetryHeader()
{
    printFormat("time_ms,line_pos,line_error,line_filt,line_steer_degps,line_diff_v,line_rot_v,"
                "motor_diff_v,target_speed,actual_speed,raw,active,current_paths,event_valid,"
                "event_paths,route_cmd,route_index,route_match\n");
}

void CommandLineInterface::printLineRunningTelemetry()
{
    if (deps_.line_follower == nullptr)
        return;

    const float    velocity_now = deps_.motion != nullptr ? deps_.motion->velocity() : 0.0f;
    const uint32_t run_ms       = deps_.line_follower->runDurationMs();
    const float    line_diff_v =
        deps_.motion != nullptr ? deps_.motion->lineDifferentialVolts() : 0.0f;
    const float line_rot_v = deps_.motion != nullptr ? deps_.motion->lineRotationVolts() : 0.0f;
    const float motor_diff_v =
        deps_.motion != nullptr ? deps_.motion->motorDifferentialVolts() : 0.0f;
    const int route_cmd = lineRouteCommandCode(deps_.line_follower->lastRouteCommand());

    printFormat("%lu,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%u,%u,%u,%u,%u,%d,%u,%u\n",
                static_cast<unsigned long>(run_ms),
                static_cast<double>(deps_.line_follower->linePosition()),
                static_cast<double>(deps_.line_follower->lineError()),
                static_cast<double>(deps_.line_follower->filteredLineError()),
                static_cast<double>(deps_.line_follower->steeringAdjustmentDegps()),
                static_cast<double>(line_diff_v), static_cast<double>(line_rot_v),
                static_cast<double>(motor_diff_v),
                static_cast<double>(deps_.line_follower->targetSpeedMmps()),
                static_cast<double>(velocity_now), deps_.line_follower->rawByte(),
                deps_.line_follower->activeMask(), deps_.line_follower->currentPathsMask(),
                deps_.line_follower->lastIntersectionValid() ? 1u : 0u,
                deps_.line_follower->lastIntersectionPathsMask(), route_cmd,
                static_cast<unsigned>(deps_.line_follower->routeIndex()),
                deps_.line_follower->lastRouteChoiceMatched() ? 1u : 0u);
}

void CommandLineInterface::printLineRunSummary()
{
    if (deps_.line_follower == nullptr)
        return;

    const uint32_t run_ms = deps_.line_follower->runDurationMs();
    if (run_ms == 0)
        return; // never started — nothing meaningful to summarize.

    const float distance_mm = deps_.motion != nullptr ? deps_.motion->position() : 0.0f;
    const float avg_v_mmps  = (run_ms > 0) ? (distance_mm / (run_ms / 1000.0f)) : 0.0f;
    const float sat_seconds = static_cast<float>(deps_.line_follower->saturationTicks()) *
                              static_cast<float>(LOOP_INTERVAL_S);

    // Modeled on PATH's "PATH done: ..." summary. Captures the peaks an
    // operator would otherwise have to scrape from the running telemetry.
    printFormat("LINE done: duration=%.2f s distance=%.0f mm avg_v=%.0f mm/s\n",
                static_cast<double>(run_ms) / 1000.0, static_cast<double>(distance_mm),
                static_cast<double>(avg_v_mmps));
    printFormat("LINE peaks: |err|=%.2f slots |steer|=%.0f deg/s v=%.0f mm/s "
                "v_min=%.0f mm/s sat=%.2f s\n",
                static_cast<double>(deps_.line_follower->peakAbsErrorSlots()),
                static_cast<double>(deps_.line_follower->peakAbsSteeringDegps()),
                static_cast<double>(deps_.line_follower->peakVelocityMmps()),
                static_cast<double>(deps_.line_follower->minVelocityMmps()),
                static_cast<double>(sat_seconds));
    printFormat("LINE counts: intersections=%lu recoveries=%lu route_index=%u/%u\n",
                static_cast<unsigned long>(deps_.line_follower->intersectionCount()),
                static_cast<unsigned long>(deps_.line_follower->recoveryCount()),
                static_cast<unsigned>(deps_.line_follower->routeIndex()),
                static_cast<unsigned>(std::strlen(deps_.line_follower->route())));
}

void CommandLineInterface::printMazeView(char mode)
{
    if (deps_.mouse == nullptr)
    {
        printFormat("Maze Mouse not initialized.\n");
        return;
    }

    if (mode == 'W' || deps_.maze_mouse == nullptr)
    {
        print(deps_.mouse->mazeString());
        return;
    }

    MazeMouse*    mouse      = deps_.maze_mouse;
    int           width      = mouse->mazeWidth();
    int           height     = mouse->mazeHeight();
    constexpr int kUnreached = 9999;

    std::vector<std::vector<int>> cost(width, std::vector<int>(height, kUnreached));
    std::queue<Cell*>             queue;
    for (const auto& goal : deps_.goal_cells)
    {
        Cell* goal_cell = mouse->cellAt(goal[0], goal[1]);
        if (goal_cell == nullptr)
            continue;
        cost[goal_cell->x()][goal_cell->y()] = 0;
        queue.push(goal_cell);
    }

    while (!queue.empty())
    {
        Cell* cell = queue.front();
        queue.pop();
        const int next_cost = cost[cell->x()][cell->y()] + 1;
        for (Cell* neighbor : mouse->cellNeighbors(cell, /*include_diagonal=*/false))
        {
            if (!mouse->canMoveBetween(cell, neighbor, /*diagonals=*/false))
                continue;
            if (next_cost >= cost[neighbor->x()][neighbor->y()])
                continue;
            cost[neighbor->x()][neighbor->y()] = next_cost;
            queue.push(neighbor);
        }
    }

    auto h_wall = [](WallState state) -> const char*
    {
        if (state == WALL)
            return "---";
        if (state == EXIT)
            return "   ";
        if (state == VIRTUAL)
            return "###";
        return "...";
    };
    auto v_wall = [](WallState state) -> char
    {
        if (state == WALL)
            return '|';
        if (state == EXIT)
            return ' ';
        if (state == VIRTUAL)
            return '#';
        return ':';
    };

    std::string out = (mode == 'C') ? "Costs:\n" : "Directions:\n";
    for (int row = height - 1; row >= 0; --row)
    {
        for (int col = 0; col < width; ++col)
        {
            Cell* cell = mouse->cellAt(col, row);
            out += '+';
            out += h_wall(cell->wallState('N'));
        }
        out += "+\n";

        for (int col = 0; col < width; ++col)
        {
            Cell* cell = mouse->cellAt(col, row);
            out += v_wall(cell->wallState('W'));

            if (mode == 'C')
            {
                char text[4];
                if (cost[col][row] >= kUnreached)
                    std::snprintf(text, sizeof(text), "###");
                else
                    std::snprintf(text, sizeof(text), "%3d", cost[col][row] % 1000);
                out += text;
            }
            else
            {
                char arrow     = ' ';
                int  best_cost = cost[col][row];
                for (Cell* neighbor : mouse->cellNeighbors(cell, /*include_diagonal=*/false))
                {
                    if (!mouse->canMoveBetween(cell, neighbor, /*diagonals=*/false))
                        continue;
                    const int neighbor_cost = cost[neighbor->x()][neighbor->y()];
                    if (neighbor_cost >= best_cost)
                        continue;
                    best_cost = neighbor_cost;
                    if (neighbor->x() > col)
                        arrow = '>';
                    else if (neighbor->x() < col)
                        arrow = '<';
                    else if (neighbor->y() > row)
                        arrow = '^';
                    else if (neighbor->y() < row)
                        arrow = 'v';
                }
                if (best_cost == 0)
                    arrow = '*';
                out += ' ';
                out += arrow;
                out += ' ';
            }
        }
        out += v_wall(mouse->cellAt(width - 1, row)->wallState('E'));
        out += '\n';
    }
    for (int col = 0; col < width; ++col)
    {
        Cell* cell = mouse->cellAt(col, 0);
        out += '+';
        out += h_wall(cell->wallState('S'));
    }
    out += "+\n";
    print(out);
}

void CommandLineInterface::printEncoderSnapshot()
{
    Motion* r = deps_.motion;
    if (r == nullptr)
    {
        printFormat("Motion not initialized.\n");
        return;
    }
    printFormat("L:%ld R:%ld P:%.1f Y:%.2f\n", static_cast<long>(r->encoder_ticks(WheelSide::LEFT)),
                static_cast<long>(r->encoder_ticks(WheelSide::RIGHT)),
                static_cast<double>(r->position()), static_cast<double>(r->yaw_deg()));
}

bool CommandLineInterface::needsTof(const char* what)
{
    if (deps_.sensor_mode == SensorMode::TOF)
        return true;

    printFormat("%s requires ToF sensor mode. Reboot Normal CLI and select 'T'.\n", what);
    return false;
}

bool CommandLineInterface::startWithGesture(bool tof_available, bool preserve_yaw)
{
    halted_ = false;
    printFormat("Waiting for start gesture (wave hand / send G / BT START)...\n");
    ToF*         front_tof = tof_available ? deps_.front_tof : nullptr;
    StartTrigger trigger   = waitForStartGesture(deps_.bluetooth, front_tof);
    if (trigger == StartTrigger::CANCELLED)
    {
        printFormat("Cancelled.\n");
        halted_ = true;
        return false;
    }

    switch (trigger)
    {
        case StartTrigger::SERIAL_G:
            printFormat("Start trigger: USB G\n");
            break;
        case StartTrigger::BT_START:
            printFormat("Start trigger: Bluetooth START\n");
            break;
        case StartTrigger::TOF_WAVE:
            printFormat("Start trigger: front ToF wave\n");
            break;
        default:
            break;
    }

    if (deps_.sensor_mode == SensorMode::TOF && deps_.motion != nullptr)
    {
        // Goal-bound stages and one-off commands start a fresh yaw frame.
        // Return stages preserve the yaw frame learned during search.
        if (preserve_yaw)
            deps_.motion->reset_drive_control();
        else
            deps_.motion->emergency_stop();
    }
    return true;
}

bool CommandLineInterface::startCenter()
{
    if (deps_.mouse == nullptr)
    {
        printFormat("Mouse not initialized.\n");
        return false;
    }

    printFormat("Start-center: %.1f mm\n", static_cast<double>(START_CENTER_DISTANCE_MM));
    return deps_.mouse->start_center() && !halted_;
}

void CommandLineInterface::stop()
{
    // UKMARS pattern: emergency_stop() resets the drive system in place.
    // Any blocking motion call will see halt_check_ return true on its next
    // 2 ms iteration and exit the busy-wait.
    if (deps_.motion != nullptr)
        deps_.motion->emergency_stop();
    if (deps_.line_follower != nullptr)
        deps_.line_follower->stop();
    halted_ = true;
    printFormat("STOP\n");
}

void CommandLineInterface::reset()
{
    if (deps_.sensor_mode == SensorMode::TOF && deps_.motion != nullptr)
        deps_.motion->emergency_stop();
    if (deps_.line_follower != nullptr)
        deps_.line_follower->reset();

    if (deps_.maze != nullptr)
        deps_.maze->reset();
    if (deps_.maze_mouse != nullptr)
        deps_.maze_mouse->reset(deps_.start_cell, "n", deps_.goal_cells);
    if (deps_.mouse != nullptr)
    {
        deps_.mouse->setPhaseColor('y');
        deps_.mouse->setUp(deps_.start_cell, deps_.goal_cells);
    }

    halted_        = false;
    last_function_ = -1;
    printFormat("RESET: search state reset\n");
}

uint8_t CommandLineInterface::read_integer(const char* line, int& value)
{
    if (line == nullptr)
        return 0;

    const char* ptr      = line;
    bool        is_minus = false;
    uint8_t     digits   = 0;

    if (*ptr == '-')
    {
        is_minus = true;
        ++ptr;
    }

    int32_t number = 0;
    while (*ptr >= '0' && *ptr <= '9')
    {
        number = 10 * number + (*ptr - '0');
        ++digits;
        ++ptr;
    }

    if (digits > 0)
        value = is_minus ? -number : number;
    return digits;
}

void CommandLineInterface::clear_input_buffer()
{
    line_index_     = 0;
    line_buffer_[0] = '\0';
}

void CommandLineInterface::prompt()
{
    printFormat("\n> ");
}

void CommandLineInterface::help()
{
    printFormat("? : this text\n");
    printFormat("X : stop motion\n");
    printFormat("W : display maze walls\n");
    printFormat("C : display maze costs\n");
    printFormat("D : display maze with directions\n");
    printFormat("B : show battery voltage\n");
    printFormat("S : show sensor readings\n");
    printFormat("E : show encoder readings\n");
    printFormat("Q : show encoder readings\n");
    printFormat("F n : Run user function n\n");
    printFormat(" 0 = ---\n");
    printFormat(" 1 = Sensor Static Calibration\n");
    printFormat(" 2 = Search to the goal and back\n");
    printFormat(" 3 = Follow a wall to the goal\n");
    printFormat(" 4 = Test SS90E Turn (starts immediately)\n");
    printFormat(" 5 = Wander\n");
    printFormat(" 6 = Test Edge Detect Position\n");
    printFormat(" 7 = Sensor Spin Calibration\n");
    printFormat(" 8 = Get Front Sensor table\n");
    printFormat(" 9 = move forward 4 cells\n");
    printFormat(" 10 = \n");
    printFormat(" 11 = \n");
    printFormat(" 12 = \n");
    printFormat(" 13 = \n");
    printFormat(" 14 = \n");
    printFormat(" 15 = \n");
    printFormat("SEARCH x y : search to location (x,y)\n");
    printFormat("S : sensor snapshot (ToF or LineSensor)\n");
    printFormat("LINE STATUS : readable line sensor snapshot\n");
    printFormat("LINE START/STOP/LEFT/RIGHT : LineSensor mode control\n");
    printFormat("LINE TUNE/SET/DEFAULTS : runtime line tuning\n");
    printFormat("HELP DEBUG : Jurababa extensions\n");
    printFormat("HELP : this text\n");
}

void CommandLineInterface::help_debug()
{
    printFormat("Jurababa debug extensions:\n");
    printFormat("STYLE [STATIONARY|SMOOTH] : select path execution style\n");
    printFormat(
        "PATH seq [spd acc omg alp] [SMOOTH] [RAW] : manual path, RAW disables heading hold\n");
    printFormat("CENTER [spd acc] : move from start pose to cell center (%.1f mm)\n",
                static_cast<double>(START_CENTER_DISTANCE_MM));
    printFormat("STARTCENTER [spd acc] : CENTER alias\n");
    printFormat("STAGE 1 : search configured goal\n");
    printFormat("STAGE 2 : return to start\n");
    printFormat("STAGE 3 : explored cardinal fast goal\n");
    printFormat("STAGE 4 : explored cardinal fast start\n");
    printFormat("STAGE 5 : explored diagonal fast goal\n");
    printFormat("SEARCHGOAL/RETURNSTART/FASTGOAL/FASTSTART/DIAGGOAL : stage aliases\n");
    printFormat("SIMRUN : STAGE 1 + STAGE 2 + STAGE 5\n");
    printFormat("S : sensor snapshot (ToF or LineSensor)\n");
    printFormat("LINE STATUS : readable line sensor snapshot\n");
    printFormat("LINE TUNE : print line KP/KD and speed envelope\n");
    printFormat("LINE SET <name> <value> : edit one runtime line tuning value\n");
    printFormat("LINE DEFAULTS : restore line tuning values from config/tuning.h\n");
    printFormat("LINE ROUTE <LFR...|CLEAR|STATUS> : route override/progress\n");
    printFormat("LINE START/STOP/LEFT/RIGHT : LineSensor mode control\n");
    printFormat("COMP : gesture-driven competition mode\n");
    printFormat("       front wave -> Stage 1+2 (search and return)\n");
    printFormat("       right wave -> Stage 3 (cardinal fast run)\n");
    printFormat("       left  wave -> Stage 5 (diagonal fast run)\n");
    printFormat("       BOOTSEL    -> abort current run, then pause\n");
    printFormat("       BOOTSEL    -> (while paused) resume to armed\n");
    printFormat("       (abort rolls back wall data; HALT/X while paused exits COMP)\n");
    printFormat("CHEESE : one-front-wave Cheese Hunt beacon search\n");
    printFormat("RESET : reset search state without rebooting\n");
    printFormat("HALT : stop motion\n");
}
