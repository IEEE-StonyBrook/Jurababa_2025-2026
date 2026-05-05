#ifndef APP_CLI_H
#define APP_CLI_H

#include <array>
#include <cstdint>
#include <vector>

#include "app/api.h" // for MotionWaiter

class API;
class Battery;
class Bluetooth;
class DriverLab;
class LineFollower;
class Maze;
class Mouse;

/**
 * @brief Sensor mode chosen at boot.
 *
 * I2C0 is shared between the front/left/right ToFs and the YahBoom line
 * sensor, so they're mutually exclusive. The boot prompt picks one and
 * Core 1 only launches in TOF mode (line-sensor mode keeps motors on
 * Core 0 for direct LineFollower control).
 */
enum class SensorMode
{
    TOF,
    LINE_SENSOR
};

constexpr int CLI_MAX_ARGC = 16;

struct Args
{
    char* argv[CLI_MAX_ARGC] = {};
    int   argc               = 0;

    void print() const;
};

/**
 * @brief UKMARS mazerunner-core-style command line interface.
 *
 * Normal CLI intentionally mirrors mazerunner-core's command surface:
 * one-character commands, `F n` function dispatch, and long commands such as
 * HELP and SEARCH. Jurababa keeps motor-safe STOP semantics for X and adds
 * STYLE for stationary/smooth motion execution.
 */
class CommandLineInterface : public MotionWaiter
{
  public:
    struct Deps
    {
        Bluetooth*                      bluetooth     = nullptr;
        Battery*                        battery       = nullptr;
        LineFollower*                   line_follower = nullptr; // null in ToF mode
        DriverLab*                      driver_lab    = nullptr;
        Maze*                           maze          = nullptr;
        Mouse*                          mouse         = nullptr;
        API*                            api           = nullptr; // FirmwareApi on hardware
        SensorMode                      sensor_mode   = SensorMode::TOF;
        std::array<int, 2>              start_cell    = {0, 0};
        std::vector<std::array<int, 2>> goal_cells    = {};
    };

    explicit CommandLineInterface(const Deps& deps);

    void greet();
    void loop();
    bool pollOnce() { return process_serial_data(); }

    bool process_serial_data();
    void process_input_line();

    // MotionWaiter implementation: waits for Core1 to complete the matching
    // command ID and interleaves `pollHaltOnly()` so HALT cuts through motion.
    void waitForMotionComplete(uint16_t command_id) override;

    void runFunction(int n) { run_function(n); }
    void run_function(int cmd);
    void help();
    void prompt();

    bool halted() const { return halted_; }

  private:
    static uint8_t read_integer(const char* line, int& value);

    // Watch only for HALT signals (Bluetooth HALT, USB 'X'/'x'). Cheap
    // enough to interleave inside `waitForMotionComplete` and inside
    // long-running maze functions.
    void pollHaltOnly();

    void handle_backspace();
    void add_to_buffer(char c);
    int  tokenise(Args& args, char* line);
    void execute_command(Args& args);
    void run_short_cmd(const Args& args);
    void run_long_cmd(const Args& args);
    void handle_search_command(const Args& args);
    void handle_style_command(const Args& args);
    void clear_input_buffer();
    void handleBluetoothCommand();

    void dumpSensorsOneShot();
    void printMazeView(char mode);
    void printEncoderSnapshot();
    void printTofSnapshot();
    bool needsTof(const char* what) const;
    bool startWithGesture(bool tof_available);
    void stop();

    Deps deps_;

    static constexpr int LINE_BUFFER_SIZE               = 96;
    char                 line_buffer_[LINE_BUFFER_SIZE] = {};
    int                  line_index_                    = 0;

    int  last_function_ = -1;
    bool halted_        = false;
};

using Cli = CommandLineInterface;

#endif
