#ifndef APP_CLI_H
#define APP_CLI_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class Mouse;
class Battery;
class Bluetooth;
class DriverLab;
class LineFollower;
class Maze;
class MazeMouse;
class Motion;
class ToF;

/**
 * @brief Sensor mode chosen at boot.
 *
 * I2C0 is shared between the front/left/right ToFs and the YahBoom line
 * sensor, so they're mutually exclusive. The boot prompt picks one. ToF
 * mode wires Motion directly into the Mouse; LineSensor mode hands motors
 * to LineFollower instead.
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
};

/**
 * @brief UKMARS mazerunner-core-style command line interface.
 *
 * Single-core: motion commands call directly into Motion via the Mouse and
 * busy-wait while a 500 Hz hardware timer advances the controller. Same
 * shape as mazerunner-core's `loop()` + Timer2 ISR split.
 */
class CommandLineInterface
{
  public:
    struct Deps
    {
        Bluetooth*                      bluetooth     = nullptr;
        Battery*                        battery       = nullptr;
        Motion*                         motion        = nullptr; // null in LineSensor mode
        ToF*                            left_tof      = nullptr; // null in LineSensor mode
        ToF*                            front_tof     = nullptr;
        ToF*                            right_tof     = nullptr;
        LineFollower*                   line_follower = nullptr; // null in ToF mode
        DriverLab*                      driver_lab    = nullptr;
        Maze*                           maze          = nullptr;
        MazeMouse*                      maze_mouse    = nullptr;
        Mouse*                          mouse         = nullptr; // FirmwareMouse on hardware
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

    void runFunction(int n) { run_function(n); }
    void run_function(int cmd);
    void help();
    void help_debug();
    void prompt();

    bool halted() const { return halted_; }

    // Halt thunk for Mouse: returns true when 'X' / HALT was received over
    // serial during a long-running motion. Mouse installs this via
    // `setHaltCheck` so blocking commands can break early.
    static bool haltCheckThunk();

  private:
    static uint8_t read_integer(const char* line, int& value);

    void handle_backspace();
    void add_to_buffer(char c);
    int  tokenise(Args& args, char* line);
    void process_line(char* line);
    void execute_command(Args& args);
    void run_short_cmd(const Args& args);
    void run_long_cmd(const Args& args);
    void handle_search_command(const Args& args);
    void handle_stage_command(const Args& args);
    void handle_style_command(const Args& args);
    void handle_path_command(const Args& args);
    void handle_center_command(const Args& args);
    void handle_line_command(const Args& args);
    void clear_input_buffer();
    void handleBluetoothCommand();
    bool run_competition_stage(int stage, bool wait_for_start);

    void dumpSensorsOneShot();
    void printLineSnapshot();
    void printMazeView(char mode);
    void printEncoderSnapshot();
    bool needsTof(const char* what);
    bool startWithGesture(bool tof_available);
    bool startCenter();
    enum class PathSegmentType
    {
        Forward,
        Turn,
        SmoothTurn
    };
    struct PathSegment
    {
        PathSegmentType type;
        float           value; // Forward: mm. Turns: deg.
    };
    bool parse_path_sequence(const Args& args, int first_param_index,
                             std::vector<PathSegment>& segments);
    bool parse_path_chunk(const char* chunk, std::vector<PathSegment>& segments);
    void append_path_forward_cells(std::vector<PathSegment>& segments, int cells);
    void append_path_turn(std::vector<PathSegment>& segments, float degrees);
    bool run_path_segments(std::vector<PathSegment>& segments, float speed_mmps, float accel_mmps2,
                           float omega_degps, float alpha_degps2, bool smooth_turns);
    bool wait_path_segment_motion();
    void stop();
    void reset();
    void print(const char* text);
    void print(const std::string& text);
    void printFormat(const char* format, ...);
    void printArgs(const Args& args);
    void drainConsole();

    // Process bluetooth/USB input non-blockingly during motion. Sets
    // halted_ if HALT/X received. Used by the Mouse halt thunk.
    void pollHaltOnly();

    Deps deps_;

    static constexpr int LINE_BUFFER_SIZE               = 96;
    char                 line_buffer_[LINE_BUFFER_SIZE] = {};
    int                  line_index_                    = 0;

    int  last_function_ = -1;
    bool halted_        = false;

    static CommandLineInterface* s_instance_;
};

using Cli = CommandLineInterface;

#endif
