#include "app/cli.h"

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "pico/stdlib.h"

#include "app/api.h"
#include "app/bluetooth.h"
#include "app/start_gesture.h"
#include "common/log.h"
#include "common/tof_wall_utils.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "config/sensors.h"
#include "config/smooth_turn.h"
#include "control/robot.h"
#include "drivers/battery.h"
#include "drivers/tof.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/path_utils.h"

namespace
{
constexpr char kBackspace = 0x08;

const char* movementStyleName(API::MovementStyle style)
{
    return style == API::MovementStyle::Smooth ? "SMOOTH" : "STATIONARY";
}

const char* tofReadingName(int16_t mm)
{
    if (mm <= 0)
        return "invalid";
    return mm >= static_cast<int16_t>(TOF_OUT_OF_RANGE_MM) ? "open" : "valid";
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
    if (deps_.api != nullptr)
        deps_.api->setHaltCheck(&CommandLineInterface::haltCheckThunk);
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
    printFormat("Sensor mode: %s\n", deps_.sensor_mode == SensorMode::TOF ? "ToF" : "LineSensor");
    print("DriverLab is a separate boot mode; Normal CLI does not run OL/STEP/MOVE/TURN.\n");
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
    // Robot::update() runs concurrently via the 500 Hz hardware timer
    // alarm registered in main.cpp.
    while (true)
    {
        process_serial_data();
        if (deps_.battery != nullptr)
            deps_.battery->update();
        if (deps_.api != nullptr)
            deps_.api->serviceSensors();

        sleep_ms(2);
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
            printTofSnapshot();
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
    if (std::strcmp(args.argv[0], "COMP") == 0)
    {
        run_competition_flow();
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
    if (deps_.api == nullptr || deps_.mouse == nullptr)
    {
        printFormat("Maze API not initialized.\n");
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

    printFormat("Search to %d,%d\n", x, y);
    std::vector<std::array<int, 2>> goals = {{x, y}};
    const bool ok = PathUtils::traversePath(deps_.api, deps_.mouse, goals, /*diagonals=*/false,
                                            /*all_explored=*/false, /*avoid_goals=*/false,
                                            /*start_at_wall_check=*/true);
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
    if (deps_.api == nullptr)
    {
        printFormat("API not initialized.\n");
        return;
    }

    if (args.argc < 2)
    {
        printFormat("STYLE: %s\n", movementStyleName(deps_.api->movementStyle()));
        return;
    }

    if (std::strcmp(args.argv[1], "SMOOTH") == 0)
    {
        deps_.api->setMovementStyle(API::MovementStyle::Smooth);
        printFormat("STYLE: SMOOTH\n");
        return;
    }

    if (std::strcmp(args.argv[1], "STATIONARY") == 0 || std::strcmp(args.argv[1], "STILL") == 0)
    {
        deps_.api->setMovementStyle(API::MovementStyle::Stationary);
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
    if (deps_.api == nullptr || deps_.robot == nullptr)
    {
        printFormat("PATH requires Robot/API in ToF CLI mode.\n");
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

    deps_.robot->begin_motion_sequence();
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
        const float        yaw_before_deg = deps_.robot->angle();

        if (segment.type == PathSegmentType::Forward)
        {
            printFormat("PATH %lu/%lu: F %.1f mm\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            deps_.robot->start_move(segment.value, speed_mmps, 0.0f, accel_mmps2);
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
            deps_.robot->turn_smooth(turn_id);
            ok = wait_path_segment_motion();
            expected_yaw_deg += params.angle_deg;
        }
        else
        {
            printFormat("PATH %lu/%lu: TURN %.1f deg\n", static_cast<unsigned long>(i + 1),
                        static_cast<unsigned long>(segments.size()),
                        static_cast<double>(segment.value));
            deps_.robot->spin_turn(segment.value, std::fabs(omega_degps), std::fabs(alpha_degps2));
            ok = wait_path_segment_motion();
            expected_yaw_deg += segment.value;
        }

        const float yaw_after_deg        = deps_.robot->angle();
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
            deps_.robot->end_motion_sequence();
            return false;
        }
    }

    deps_.robot->stop();
    const float final_yaw   = deps_.robot->angle();
    const float final_error = normalizeYawDelta(expected_yaw_deg - final_yaw);
    printFormat("PATH done: segments=%lu forward=%.1f mm (%.2f cells) expected_yaw=%.2f "
                "final_yaw=%.2f error=%+.2f deg\n",
                static_cast<unsigned long>(segments.size()), static_cast<double>(total_forward_mm),
                static_cast<double>(total_forward_mm / CELL_SIZE_MM),
                static_cast<double>(expected_yaw_deg), static_cast<double>(final_yaw),
                static_cast<double>(final_error));
    deps_.robot->end_motion_sequence();
    return true;
}

bool CommandLineInterface::wait_path_segment_motion()
{
    if (deps_.robot == nullptr)
        return false;

    while (!deps_.robot->move_finished() || !deps_.robot->turn_finished())
    {
        if (deps_.api != nullptr)
            deps_.api->serviceSensors();
        pollHaltOnly();
        if (halted_)
        {
            deps_.robot->emergency_stop();
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
    if (deps_.robot != nullptr)
        deps_.robot->reset_drive_system();
    if (deps_.api == nullptr || !deps_.api->move_physical(START_CENTER_DISTANCE_MM, speed, accel))
        printFormat("CENTER failed.\n");
}

bool CommandLineInterface::run_competition_stage(int stage, bool wait_for_start)
{
    if (!needsTof("STAGE"))
        return false;
    if (deps_.api == nullptr || deps_.mouse == nullptr)
    {
        printFormat("Maze API not initialized.\n");
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
            deps_.api->setPhaseColor('y');
            return PathUtils::traversePath(deps_.api, deps_.mouse, deps_.goal_cells,
                                           /*diagonals=*/false, /*all_explored=*/false,
                                           /*avoid_goals=*/false,
                                           /*start_at_wall_check=*/true);

        case 2:
        {
            printFormat("Stage 2: iterative A* return to start.\n");
            deps_.api->setPhaseColor('c');
            std::vector<std::array<int, 2>> goals = {deps_.start_cell};
            return PathUtils::traversePath(deps_.api, deps_.mouse, goals, /*diagonals=*/false,
                                           /*all_explored=*/false, /*avoid_goals=*/false);
        }

        case 3:
            printFormat("Stage 3: explored-only cardinal fast run to goal.\n");
            deps_.api->setPhaseColor('g');
            return PathUtils::traverseExploredPath(deps_.api, deps_.mouse, deps_.goal_cells);

        case 4:
        {
            printFormat("Stage 4: explored-only cardinal fast return to start.\n");
            deps_.api->setPhaseColor('c');
            std::vector<std::array<int, 2>> goals = {deps_.start_cell};
            return PathUtils::traverseExploredPath(deps_.api, deps_.mouse, goals);
        }

        case 5:
            printFormat("Stage 5: explored-only diagonal fast run to goal.\n");
            deps_.api->setPhaseColor('G');
            return PathUtils::traverseExploredDiagonalPath(deps_.api, deps_.mouse,
                                                           deps_.goal_cells);
    }

    return false;
}

void CommandLineInterface::run_competition_flow()
{
    if (!needsTof("COMP"))
        return;
    if (deps_.api == nullptr || deps_.mouse == nullptr)
    {
        printFormat("Maze API not initialized.\n");
        return;
    }
    if (!startWithGesture(true))
        return;

    printFormat("Running competition stages 1..5.\n");
    for (int stage = 1; stage <= 5; ++stage)
    {
        if (!run_competition_stage(stage, /*wait_for_start=*/false))
        {
            printFormat("Competition stopped at stage %d.\n", stage);
            return;
        }
    }
    printFormat("Competition flow complete.\n");
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
            dumpSensorsOneShot();
            break;

        case 2:
            if (!needsTof("Search maze"))
                break;
            if (deps_.api == nullptr || deps_.mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            if (deps_.api != nullptr)
                deps_.api->setPhaseColor('y');
            printFormat("Searching maze...\n");
            printFormat(PathUtils::traversePath(deps_.api, deps_.mouse, deps_.goal_cells,
                                                /*diagonals=*/false, /*all_explored=*/false,
                                                /*avoid_goals=*/false,
                                                /*start_at_wall_check=*/true)
                            ? "Search done.\n"
                            : "Search failed.\n");
            break;

        case 3:
            if (!needsTof("Follow to start"))
                break;
            if (deps_.api == nullptr || deps_.mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            printFormat("Follow to start...\n");
            {
                std::vector<std::array<int, 2>> goals = {deps_.start_cell};
                printFormat(PathUtils::traversePath(deps_.api, deps_.mouse, goals,
                                                    /*diagonals=*/false, /*all_explored=*/false,
                                                    /*avoid_goals=*/false)
                                ? "Follow done.\n"
                                : "Follow failed.\n");
            }
            break;

        case 4:
            if (!needsTof("Test SS90E Turn"))
                break;
            if (deps_.api == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            deps_.api->turn_right();
            printFormat("SS90E right done.\n");
            break;

        case 5:
            printFormat("Wander is not implemented on Jurababa.\n");
            break;

        case 6:
            if (!needsTof("Edge detect position test"))
                break;
            dumpSensorsOneShot();
            break;

        case 7:
            printEncoderSnapshot();
            break;

        case 8:
            printTofSnapshot();
            break;

        case 9:
            if (!needsTof("Move forward 4 cells"))
                break;
            if (deps_.api == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            deps_.api->moveForward(4);
            printFormat("Forward 4 cells done.\n");
            break;

        case 10:
            if (deps_.battery == nullptr)
            {
                printFormat("Battery monitor not initialized.\n");
                break;
            }
            printFormat("Battery: %.2f V\n", deps_.battery->voltage());
            break;

        default:
            stop();
            break;
    }
}

void CommandLineInterface::dumpSensorsOneShot()
{
    Robot* r = deps_.robot;
    if (r != nullptr)
    {
        printFormat("ToF L=%d mm F=%d mm R=%d mm  yaw=%.1f deg\n",
                    static_cast<int>(r->leftDistance()), static_cast<int>(r->frontDistance()),
                    static_cast<int>(r->rightDistance()), static_cast<double>(r->angle()));
    }
    else
    {
        printFormat("ToF: robot not initialized\n");
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

void CommandLineInterface::printMazeView(char mode)
{
    if (deps_.api == nullptr)
    {
        printFormat("Maze API not initialized.\n");
        return;
    }

    if (mode == 'C')
        printFormat("Cost view is not implemented yet; printing maze walls instead.\n");
    if (mode == 'D')
        printFormat("Direction view is not implemented yet; printing maze walls instead.\n");
    print(deps_.api->mazeString());
}

void CommandLineInterface::printEncoderSnapshot()
{
    Robot* r = deps_.robot;
    if (r == nullptr)
    {
        printFormat("Robot not initialized.\n");
        return;
    }
    printFormat("position=%.1f mm velocity=%.1f mm/s yaw=%.2f deg omega=%.2f deg/s\n",
                static_cast<double>(r->position()), static_cast<double>(r->velocity()),
                static_cast<double>(r->angle()), static_cast<double>(r->omega()));
}

void CommandLineInterface::printTofSnapshot()
{
    if (!needsTof("Q"))
        return;

    Robot* r = deps_.robot;
    if (r == nullptr)
    {
        printFormat("Robot not initialized.\n");
        return;
    }

    const int16_t l_mm = static_cast<int16_t>(r->leftDistance());
    const int16_t f_mm = static_cast<int16_t>(r->frontDistance());
    const int16_t r_mm = static_cast<int16_t>(r->rightDistance());

    const tof_wall::WallState wall_state = tof_wall::evaluate(l_mm, f_mm, r_mm);

    printFormat("ToF L=%d mm (%s) F=%d mm (%s) R=%d mm (%s)\n", l_mm, tofReadingName(l_mm), f_mm,
                tofReadingName(f_mm), r_mm, tofReadingName(r_mm));
    printFormat("Walls L=%d F=%d R=%d  thresholds L=%.1f F=%.1f R=%.1f  open=%d\n",
                wall_state.left_wall, wall_state.front_wall, wall_state.right_wall,
                static_cast<double>(TOF_LEFT_WALL_THRESHOLD_MM),
                static_cast<double>(TOF_FRONT_WALL_THRESHOLD_MM),
                static_cast<double>(TOF_RIGHT_WALL_THRESHOLD_MM),
                static_cast<int>(TOF_OUT_OF_RANGE_MM));
    if (wall_state.side_error_valid)
    {
        const float steering_preview =
            wall_state.steering_allowed
                ? tof_wall::steeringAdjustmentDegps(wall_state.side_error_norm, 0.0f)
                : 0.0f;
        printFormat("Side src=%s err=%.1f norm Lerr=%.1f Rerr=%.1f preview=%.1f deg/s allowed=%d "
                    "front_blocked=%d\n",
                    tof_wall::sourceName(wall_state.source),
                    static_cast<double>(wall_state.side_error_norm),
                    static_cast<double>(wall_state.left_error_norm),
                    static_cast<double>(wall_state.right_error_norm),
                    static_cast<double>(steering_preview), wall_state.steering_allowed,
                    wall_state.front_blocked);
    }
    else
    {
        printFormat("Side src=NONE err=unavailable preview=0.0 deg/s front_blocked=%d\n",
                    wall_state.front_blocked);
    }
    const tof_wall::WallState live_state = r->wallSteeringState();
    printFormat("Cal Lmm=%.1f Rmm=%.1f nominal=%.1f  live src=%s err=%.1f norm adjust=%.1f deg/s "
                "allowed=%d\n",
                static_cast<double>(TOF_LEFT_CALIBRATION_MM),
                static_cast<double>(TOF_RIGHT_CALIBRATION_MM),
                static_cast<double>(TOF_SIDE_NOMINAL), tof_wall::sourceName(live_state.source),
                static_cast<double>(live_state.side_error_norm),
                static_cast<double>(r->wallSteeringAdjustmentDegps()), live_state.steering_allowed);
    printFormat("Yaw current=%.2f deg\n", static_cast<double>(r->angle()));

    if (deps_.mouse != nullptr)
    {
        Cell* cell = deps_.mouse->currentCell();
        if (cell != nullptr)
        {
            printFormat("Mouse cell=(%d,%d) heading=%s cell_walls N=%d E=%d S=%d W=%d\n", cell->x(),
                        cell->y(), deps_.mouse->currentDirection().c_str(), cell->hasWall('N'),
                        cell->hasWall('E'), cell->hasWall('S'), cell->hasWall('W'));
        }
    }
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

    if (deps_.sensor_mode == SensorMode::TOF && deps_.robot != nullptr)
    {
        // UKMARS pattern: clear any prior motion before starting a new run.
        deps_.robot->emergency_stop();
    }
    return true;
}

bool CommandLineInterface::startCenter()
{
    if (deps_.api == nullptr)
    {
        printFormat("API not initialized.\n");
        return false;
    }

    printFormat("Start-center: %.1f mm\n", static_cast<double>(START_CENTER_DISTANCE_MM));
    return deps_.api->start_center() && !halted_;
}

void CommandLineInterface::stop()
{
    // UKMARS pattern: emergency_stop() resets the drive system in place.
    // Any blocking motion call will see halt_check_ return true on its next
    // 2 ms iteration and exit the busy-wait.
    if (deps_.robot != nullptr)
        deps_.robot->emergency_stop();
    halted_ = true;
    printFormat("STOP\n");
}

void CommandLineInterface::reset()
{
    if (deps_.sensor_mode == SensorMode::TOF && deps_.robot != nullptr)
        deps_.robot->emergency_stop();

    if (deps_.maze != nullptr)
        deps_.maze->reset();
    if (deps_.mouse != nullptr)
        deps_.mouse->reset(deps_.start_cell, "n", deps_.goal_cells);
    if (deps_.api != nullptr)
    {
        deps_.api->clearWallSample();
        deps_.api->setPhaseColor('y');
        deps_.api->setUp(deps_.start_cell, deps_.goal_cells);
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
    printFormat("C : cost view placeholder; prints maze walls\n");
    printFormat("D : direction view placeholder; prints maze walls\n");
    printFormat("B : show battery voltage\n");
    printFormat("S : show combined sensor readings\n");
    printFormat("E : show encoder/IMU readings\n");
    printFormat("Q : show ToF readings, wall decisions, and side steering error\n");
    printFormat("F n : Run user function n\n");
    printFormat(" 0 = ---\n");
    printFormat(" 1 = Sensor Static Calibration\n");
    printFormat(" 2 = Search to the goal and back\n");
    printFormat(" 3 = Follow to start\n");
    printFormat(" 4 = Test SS90E Turn\n");
    printFormat(" 5 = Wander\n");
    printFormat(" 6 = Test Edge Detect Position\n");
    printFormat(" 7 = Sensor Spin Calibration\n");
    printFormat(" 8 = Get Front Sensor table\n");
    printFormat(" 9 = move forward 4 cells\n");
    printFormat("SEARCH x y : search to location (x,y)\n");
    printFormat("STAGE n : run competition stage 1..5\n");
    printFormat("COMP : run stages 1..5\n");
    printFormat("STYLE [STATIONARY|SMOOTH] : select path execution style\n");
    printFormat("PATH seq [spd acc omg alp smooth] : blind physical path, smooth default=0\n");
    printFormat("CENTER [spd acc] : move from start wall-check pose to cell center (%.1f mm)\n",
                static_cast<double>(START_CENTER_DISTANCE_MM));
    printFormat("RESET : reset search state without rebooting\n");
    printFormat("HELP : this text\n");
}
