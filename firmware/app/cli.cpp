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
#include "drivers/battery.h"
#include "maze/maze.h"
#include "maze/mouse.h"
#include "navigation/flood_fill.h"
#include "navigation/path_utils.h"

namespace
{
constexpr char kBackspace = 0x08;

const char* movementStyleName(API::MovementStyle style)
{
    return style == API::MovementStyle::Smooth ? "SMOOTH" : "STATIONARY";
}
} // namespace

void Args::print() const
{
    for (int i = 0; i < argc; ++i)
        printf("%s ", argv[i]);
    printf("\n");
}

CommandLineInterface::CommandLineInterface(const Deps& deps) : deps_(deps)
{
    if (deps_.api != nullptr)
        deps_.api->setMotionWaiter(this);
}

void CommandLineInterface::greet()
{
    printf("\n");
    printf("==========================================\n");
    printf("  Jurababa Micromouse -- UKMARS CLI\n");
    printf("==========================================\n");
    printf("Sensor mode: %s\n", deps_.sensor_mode == SensorMode::TOF ? "ToF" : "LineSensor");
    printf("DriverLab is a separate boot mode; Normal CLI does not run OL/STEP/MOVE/TURN.\n");
    help();
    prompt();
}

void CommandLineInterface::loop()
{
    while (true)
    {
        process_serial_data();
        sleep_ms(2);
    }
}

bool CommandLineInterface::process_serial_data()
{
    handleBluetoothCommand();

    if (deps_.battery != nullptr && deps_.sensor_mode != SensorMode::TOF)
        deps_.battery->update();

    bool processed = false;
    while (true)
    {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c < 0)
            break;

        char ch = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (ch == '\r' || ch == '\n')
        {
            printf("\n");
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
    printf("\b \b");
}

void CommandLineInterface::add_to_buffer(char c)
{
    if (line_index_ >= LINE_BUFFER_SIZE - 1)
        return;

    putchar(c);
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
    Args args;
    if (tokenise(args, line_buffer_) > 0)
        execute_command(args);

    clear_input_buffer();
    prompt();
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
                printf("F expects a function number.\n");
            }
            break;
        }
        default:
            printf("UNKNOWN COMMAND: ");
            args.print();
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
    if (std::strcmp(args.argv[0], "STYLE") == 0)
    {
        handle_style_command(args);
        return;
    }
    if (std::strcmp(args.argv[0], "HALT") == 0)
    {
        stop();
        return;
    }

    printf("UNKNOWN COMMAND: ");
    args.print();
}

void CommandLineInterface::handle_search_command(const Args& args)
{
    if (!needsTof("SEARCH"))
        return;
    if (deps_.api == nullptr || deps_.mouse == nullptr)
    {
        printf("Maze API not initialized.\n");
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

    printf("Search to %d,%d\n", x, y);
    std::vector<std::array<int, 2>> goals = {{x, y}};
    PathUtils::traversePath(deps_.api, deps_.mouse, goals, /*diagonals=*/false,
                            /*all_explored=*/false, /*avoid_goals=*/false);
}

void CommandLineInterface::handle_style_command(const Args& args)
{
    if (deps_.api == nullptr)
    {
        printf("API not initialized.\n");
        return;
    }

    if (args.argc < 2)
    {
        printf("STYLE: %s\n", movementStyleName(deps_.api->movementStyle()));
        return;
    }

    if (std::strcmp(args.argv[1], "SMOOTH") == 0)
    {
        deps_.api->setMovementStyle(API::MovementStyle::Smooth);
        printf("STYLE: SMOOTH\n");
        return;
    }

    if (std::strcmp(args.argv[1], "STATIONARY") == 0 || std::strcmp(args.argv[1], "STILL") == 0)
    {
        deps_.api->setMovementStyle(API::MovementStyle::Stationary);
        printf("STYLE: STATIONARY\n");
        return;
    }

    printf("STYLE expects STATIONARY or SMOOTH.\n");
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
                bt->write("BT START: no last function. Use F n first.\r\n");
            }
            break;
        case Bluetooth::Command::HALT:
            LOG_INFO("BT HALT");
            stop();
            break;
        case Bluetooth::Command::RESET:
            LOG_INFO("BT RESET: stopping. Reboot for full Maze reset.");
            stop();
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
    if (bt != nullptr && bt->hasCommand())
    {
        Bluetooth::Command cmd = bt->command();
        if (cmd == Bluetooth::Command::HALT)
            stop();
    }
}

void CommandLineInterface::waitForMotionComplete(uint16_t command_id)
{
    if (command_id == 0)
        return;

    while (MotionState::completed_command_id != command_id)
    {
        pollHaltOnly();
        if (halted_)
        {
            return;
        }
        sleep_ms(2);
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
            printf("Searching maze...\n");
            FloodFill::explore(*deps_.mouse, *deps_.api, /*diagonals=*/false);
            printf("Search done.\n");
            break;

        case 3:
            if (!needsTof("Follow to start"))
                break;
            if (deps_.api == nullptr || deps_.mouse == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            printf("Follow to start...\n");
            PathUtils::setAllExplored(deps_.mouse);
            {
                std::vector<std::array<int, 2>> goals = {deps_.start_cell};
                PathUtils::traversePath(deps_.api, deps_.mouse, goals, /*diagonals=*/false,
                                        /*all_explored=*/true,
                                        /*avoid_goals=*/false);
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
            printf("SS90E right done.\n");
            break;

        case 5:
            printf("Wander is not implemented on Jurababa.\n");
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
        {
            SensorData snap;
            SensorHub::snapshot(snap);
            printf("Front ToF: %d mm\n", snap.tof_front_mm);
            break;
        }

        case 9:
            if (!needsTof("Move forward 4 cells"))
                break;
            if (deps_.api == nullptr)
                break;
            if (!startWithGesture(true))
                break;
            deps_.api->moveForward(4);
            printf("Forward 4 cells done.\n");
            break;

        case 10:
            if (deps_.battery == nullptr)
            {
                printf("Battery monitor not initialized.\n");
                break;
            }
            printf("Battery: %.2f V\n", deps_.battery->voltage());
            break;

        default:
            stop();
            break;
    }
}

void CommandLineInterface::dumpSensorsOneShot()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    printf("L=%d mm  F=%d mm  R=%d mm  yaw=%.1f deg  encL=%ld encR=%ld\n", snap.tof_left_mm,
           snap.tof_front_mm, snap.tof_right_mm, static_cast<double>(snap.imu_yaw),
           static_cast<long>(snap.left_encoder), static_cast<long>(snap.right_encoder));
    if (deps_.battery != nullptr)
        printf("Battery: %.2f V\n", deps_.battery->voltage());
}

void CommandLineInterface::printMazeView(char mode)
{
    if (deps_.api == nullptr)
    {
        printf("Maze API not initialized.\n");
        return;
    }

    if (mode == 'C')
        printf("Cost view is not stored separately; printing walls.\n");
    if (mode == 'D')
        printf("Direction view is not stored separately; printing walls.\n");
    deps_.api->printMaze();
}

void CommandLineInterface::printEncoderSnapshot()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    printf("L:%ld R:%ld P:encoder-mm-unavailable A:%.2f\n", static_cast<long>(snap.left_encoder),
           static_cast<long>(snap.right_encoder), static_cast<double>(snap.imu_yaw));
}

bool CommandLineInterface::needsTof(const char* what) const
{
    if (deps_.sensor_mode == SensorMode::TOF)
        return true;

    printf("%s requires ToF sensor mode. Reboot Normal CLI and select 'T'.\n", what);
    return false;
}

bool CommandLineInterface::startWithGesture(bool tof_available)
{
    printf("Waiting for start gesture (wave hand / send G / BT START)...\n");
    StartTrigger trigger = waitForStartGesture(deps_.bluetooth, tof_available);
    if (trigger == StartTrigger::CANCELLED)
    {
        printf("Cancelled.\n");
        halted_ = true;
        return false;
    }
    return true;
}

void CommandLineInterface::stop()
{
    if (deps_.sensor_mode == SensorMode::TOF)
        CommandHub::requestStop();
    halted_ = true;
    printf("STOP\n");
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
    printf("\n> ");
}

void CommandLineInterface::help()
{
    printf("? : this text\n");
    printf("X : stop motion\n");
    printf("W : display maze walls\n");
    printf("C : display maze costs\n");
    printf("D : display maze with directions\n");
    printf("B : show battery voltage\n");
    printf("S : show sensor readings\n");
    printf("E : show encoder/IMU readings\n");
    printf("Q : show encoder/IMU readings\n");
    printf("F n : Run user function n\n");
    printf(" 0 = ---\n");
    printf(" 1 = Sensor Static Calibration\n");
    printf(" 2 = Search to the goal and back\n");
    printf(" 3 = Follow to start\n");
    printf(" 4 = Test SS90E Turn\n");
    printf(" 5 = Wander\n");
    printf(" 6 = Test Edge Detect Position\n");
    printf(" 7 = Sensor Spin Calibration\n");
    printf(" 8 = Get Front Sensor table\n");
    printf(" 9 = move forward 4 cells\n");
    printf("SEARCH x y : search to location (x,y)\n");
    printf("STYLE [STATIONARY|SMOOTH] : select path execution style\n");
    printf("HELP : this text\n");
}
