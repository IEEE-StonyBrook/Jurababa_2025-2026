#ifndef APP_API_H
#define APP_API_H

#include <array>
#include <string>
#include <vector>

class Mouse;
class Drivetrain;

/**
 * @brief Strategy interface for blocking until Core 1 finishes a motion.
 *
 * Implemented by `Cli` on hardware (polls `MotionState::active` while
 * watching for HALT). Sim build leaves the pointer null — simulator commands
 * are synchronous already.
 */
class MotionWaiter
{
  public:
    virtual ~MotionWaiter()              = default;
    virtual void waitForMotionComplete() = 0;
};

/**
 * @brief High-level maze navigation API
 *
 * Provides movement commands, wall sensing, and maze visualization.
 * Bridges between navigation algorithms and hardware control.
 *
 * `wallLeft/Front/Right` are virtual so `FirmwareApi` can read real ToF
 * snapshots on hardware while the sim build keeps using bare `API`.
 */
class API
{
  public:
    explicit API(Mouse* mouse);
    virtual ~API() = default;

    // Maze dimensions
    int mazeWidth();
    int mazeHeight();

    // Wall queries
    virtual bool wallLeft();
    virtual bool wallFront();
    virtual bool wallRight();

    // Movement commands
    void moveForwardHalf();
    void moveForward();
    void moveForward(int steps);
    void ghostMoveForward(int steps);

    void turnLeft45();
    void turnLeft90();
    void turnRight45();
    void turnRight90();
    void turn(int degrees);

    // Arc turns (smooth turns with forward motion)
    void arcTurnLeft90();
    void arcTurnRight90();
    void arcTurnLeft45();
    void arcTurnRight45();

    // Execute command sequence like "L#F#R#F"
    void executeSequence(const std::string& sequence);

    // Maze wall state
    void setWall(int x, int y, const std::string& dir);
    void clearWall(int x, int y, const std::string& dir);

    // Cell visualization
    void setColor(int x, int y, char color);
    void clearColor(int x, int y);
    void clearAllColor();
    void setText(int x, int y, const std::string& text);
    void clearText(int x, int y);
    void clearAllText();

    // Phase tracking for visualization
    void setPhaseColor(char color);
    char phaseColor() const;

    // Setup
    void setUp(std::array<int, 2> start, std::vector<std::array<int, 2>> goals);
    void printMaze();

    // Pico-specific
    void goToCenterFromEdge();

    // Inject a motion-complete waiter (set by Cli on hardware; null in sim).
    // When set, every CommandHub::send call blocks here until Core 1 finishes
    // the motion — this is what keeps the 8-deep multicore FIFO from filling
    // and deadlocking on `multicore_fifo_push_blocking`.
    void setMotionWaiter(MotionWaiter* waiter) { motion_waiter_ = waiter; }

    bool run_on_simulator = false;

  protected:
    void waitForMotion()
    {
        if (motion_waiter_ != nullptr)
            motion_waiter_->waitForMotionComplete();
    }

  private:
    std::string simulatorResponse(const std::string& cmd);
    int         simulatorInt(const std::string& cmd);
    bool        simulatorBool(const std::string& cmd);
    std::string printMazeRow(int row);

    Mouse*        mouse_;
    char          phase_color_   = 'y';
    MotionWaiter* motion_waiter_ = nullptr;
};

#endif
