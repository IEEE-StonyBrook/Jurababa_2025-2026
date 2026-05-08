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
    print("  Jurababa Micromouse -- UKMARS CLI\n");
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
    // alarm registered in main.cpp.
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
                                             float alpha_degps2, bool smooth_turns)
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

    printFormat("PATH segments=%lu speed=%.1f accel=%.1f omega=%.1f alpha=%.1f smooth=%s\n",
                static_cast<unsigned long>(segments.size()), static_cast<double>(speed_mmps),
                static_cast<double>(accel_mmps2), static_cast<double>(omega_degps),
                static_cast<double>(alpha_degps2), smooth_turns ? "true" : "false");

    for (size_t i = 0; i < segments.size(); ++i)
    {
        const PathSegment& segment        = segments[i];
        bool               ok             = true;
        const float        yaw_before_deg = deps_.motion->angle();

        if (segment.type == PathSegmentType::Forward)
        {
            printFormat("PATH %lu/%lu: F %.1f mm\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            deps_.motion->start_move(segment.value, speed_mmps, 0.0f, accel_mmps2);
            ok = wait_path_segment_motion();
            total_forward_mm += segment.value;
        }
        else if (segment.type == PathSegmentType::SmoothTurn)
        {
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
            printFormat("PATH %lu/%lu: TURN %.1f deg\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            deps_.motion->spin_turn(segment.value, std::fabs(omega_degps), std::fabs(alpha_degps2));
            ok = wait_path_segment_motion();
            expected_yaw_deg += segment.value;
        }

        const float yaw_after_deg        = deps_.motion->angle();
        const float actual_delta_deg     = normalizeYawDelta(yaw_after_deg - yaw_before_deg);
        const float segment_error_deg    = normalizeYawDelta(expected_yaw_deg - yaw_after_deg);
        const float expected_yaw_wrapped = normalizeYawDelta(expected_yaw_deg);
        printFormat("PATH %lu/%lu result: yaw_before=%.2f yaw_after=%.2f delta=%+.2f "
                    "expected_yaw=%.2f yaw_error=%+.2f\n",
                    static_cast<unsigned long>(i + 1), static_cast<unsigned long>(segments.size()),
                    static_cast<double>(yaw_before_deg), static_cast<double>(yaw_after_deg),
                    static_cast<double>(actual_delta_deg),
                    static_cast<double>(expected_yaw_wrapped),
                    static_cast<double>(segment_error_deg));

        if (!ok || halted_)
        {
            printFormat("PATH stopped at segment %lu.\n", static_cast<unsigned long>(i + 1));
            deps_.motion->end_motion_sequence();
            return false;
        }
    }

    deps_.motion->stop();
    const float final_yaw   = deps_.motion->angle();
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
    int  arg_limit    = args.argc;
    if (args.argc > 2 && parsePathSmoothFlag(args.argv[args.argc - 1], smooth_turns))
        arg_limit = args.argc - 1;

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
                    "[alpha_degps2] [smooth_bool]\n");
        return;
    }

    run_path_segments(segments, speed, accel, omega, alpha, smooth_turns);
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

    if (std::strcmp(args.argv[1], "START") == 0)
    {
        deps_.line_follower->startFollowing();
        printFormat("LINE START\n");
        return;
    }

    if (std::strcmp(args.argv[1], "STOP") == 0)
    {
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
            printFormat("LINE ROUTE usage: LINE ROUTE <LFR...|CLEAR>\n");
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

    printFormat("LINE usage: LINE [STATUS|START|STOP|LEFT|RIGHT|ROUTE]\n");
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

    if (wait_for_start && !startWithGesture(true))
        return false;

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
            printFormat("Stage 3: explored-only cardinal fast run to goal.\n");
            deps_.mouse->setPhaseColor('g');
            return PathUtils::traverseExploredPath(deps_.mouse, deps_.maze_mouse, deps_.goal_cells);

        case 4:
        {
            printFormat("Stage 4: explored-only cardinal fast return to start.\n");
            deps_.mouse->setPhaseColor('c');
            std::vector<std::array<int, 2>> goals = {deps_.start_cell};
            return PathUtils::traverseExploredPath(deps_.mouse, deps_.maze_mouse, goals);
        }

        case 5:
            printFormat("Stage 5: explored-only diagonal fast run to goal.\n");
            deps_.mouse->setPhaseColor('G');
            return PathUtils::traverseExploredDiagonalPath(deps_.mouse, deps_.maze_mouse,
                                                           deps_.goal_cells);
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
            if (last_function_ >= 0)
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
            stop();
    }
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
            if (!startWithGesture(true))
                break;
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
        printFormat("ToF L=%d mm F=%d mm R=%d mm  yaw=%.1f deg\n",
                    static_cast<int>(r->leftDistance()), static_cast<int>(r->frontDistance()),
                    static_cast<int>(r->rightDistance()), static_cast<double>(r->angle()));
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
        case LineFollower::State::AdvancingBeforeTurn:
            name = "ADVANCE_TURN";
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
    printFormat("Line raw=0x%02X active=0x%02X present=%d lost=%d pos=%.2f err=%.2f "
                "filt=%.2f steer=%.1f deg/s motion_steer=%.1f valid=%d state=%s route=%s next=%u\n",
                deps_.line_follower->rawByte(), deps_.line_follower->activeMask(),
                deps_.line_follower->linePresent(), deps_.line_follower->lineLost(),
                static_cast<double>(deps_.line_follower->linePosition()),
                static_cast<double>(deps_.line_follower->lineError()),
                static_cast<double>(deps_.line_follower->filteredLineError()),
                static_cast<double>(deps_.line_follower->steeringAdjustmentDegps()),
                static_cast<double>(motion_steering), motion_steering_valid, name,
                deps_.line_follower->route(),
                static_cast<unsigned>(deps_.line_follower->routeIndex()));
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
    printFormat("L:%ld R:%ld P:%.1f A:%.2f\n", static_cast<long>(r->encoder_ticks(WheelSide::LEFT)),
                static_cast<long>(r->encoder_ticks(WheelSide::RIGHT)),
                static_cast<double>(r->position()), static_cast<double>(r->angle()));
}

bool CommandLineInterface::needsTof(const char* what)
{
    if (deps_.sensor_mode == SensorMode::TOF)
        return true;

    printFormat("%s requires ToF sensor mode. Reboot Normal CLI and select 'T'.\n", what);
    return false;
}

bool CommandLineInterface::startWithGesture(bool tof_available)
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
        // UKMARS pattern: clear any prior motion before starting a new run.
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
    printFormat(" 4 = Test SS90E Turn\n");
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
    printFormat("LINE [STATUS|START|STOP|LEFT|RIGHT] : LineSensor mode control\n");
    printFormat("HELP DEBUG : Jurababa extensions\n");
    printFormat("HELP : this text\n");
}

void CommandLineInterface::help_debug()
{
    printFormat("Jurababa debug extensions:\n");
    printFormat("STYLE [STATIONARY|SMOOTH] : select path execution style\n");
    printFormat("PATH seq [spd acc omg alp smooth] : blind physical path, smooth default=0\n");
    printFormat("CENTER [spd acc] : move from start pose to cell center (%.1f mm)\n",
                static_cast<double>(START_CENTER_DISTANCE_MM));
    printFormat("STARTCENTER [spd acc] : CENTER alias\n");
    printFormat("STAGE n : run competition stage 1..5\n");
    printFormat("RESET : reset search state without rebooting\n");
    printFormat("HALT : stop motion\n");
}
