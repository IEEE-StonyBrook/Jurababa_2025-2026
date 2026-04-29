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

/**
 * @brief UKMARS mazerunner-core-style unified command loop.
 *
 * Replaces the three parallel `runNormalMode / runDriverLabMode /
 * runLineFollowingMode` paths with a single dispatch table. Reads lines
 * from USB-CDC stdin, watches Bluetooth for single-byte commands, and
 * routes:
 *   - integer N           → `runFunction(N)` (the dispatch table)
 *   - short single-char    → built-in (?, B, H, X, RUN n, G)
 *   - alpha tokens (OL, …) → DriverLab::executeLine
 *
 * Implements `MotionWaiter` so `API::waitForMotion` can spin on
 * `MotionState::active` while keeping HALT responsive (~20 ms worst
 * case).
 */
class Cli : public MotionWaiter
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

    explicit Cli(const Deps& deps);

    void greet();    // banner + numbered-function help
    void loop();     // never returns
    bool pollOnce(); // returns true if at least one line was processed

    // MotionWaiter implementation: spins on `MotionState::active`,
    // interleaves `pollHaltOnly()` so HALT cuts through long motions.
    void waitForMotionComplete() override;

    // The numbered dispatch table — `Cli::loop` calls this when the user
    // types an integer. Each numbered behavior runs its own start-gesture
    // handshake before sending motion (mazerunner-core convention).
    void runFunction(int n);

    bool halted() const { return halted_; }

  private:
    // Watch only for HALT signals (Bluetooth HALT, USB 'X'/'x'). Cheap
    // enough to interleave inside `waitForMotionComplete` and inside
    // long-running maze functions.
    void pollHaltOnly();

    // Read one full line from USB stdin into `line_buffer_`, returns true
    // when a line is complete (terminator stripped). Non-blocking.
    bool readLineNonBlocking();

    void executeLine();
    void handleBluetoothCommand();

    void dumpSensorsOneShot();
    void runLineFollowEventLoop();
    void printEncoderDelta(int32_t l_before, int32_t r_before, int32_t l_after, int32_t r_after);

    Deps deps_;

    static constexpr int LINE_BUFFER_SIZE               = 96;
    char                 line_buffer_[LINE_BUFFER_SIZE] = {};
    int                  line_index_                    = 0;

    int  last_function_ = -1;
    bool halted_        = false;
};

#endif
