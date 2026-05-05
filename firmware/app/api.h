#ifndef APP_API_H
#define APP_API_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class Mouse;

/**
 * @brief Strategy interface for blocking until Core 1 finishes a motion.
 *
 * Implemented by `Cli` on hardware (waits for the exact command ID Core1
 * completes while watching for HALT). Sim build leaves the pointer null —
 * simulator commands are synchronous already.
 */
class MotionWaiter
{
  public:
    virtual ~MotionWaiter()                                 = default;
    virtual void waitForMotionComplete(uint16_t command_id) = 0;
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
    enum class MovementStyle
    {
        Stationary,
        Smooth
    };

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
    void move_mm(float distance_mm);
    void start_center();
    void moveForward();
    void moveForward(int steps);
    void ghostMoveForward(int steps);

    void turnLeft45();
    void turnLeft90();
    void turnRight45();
    void turnRight90();
    void turn(int degrees);

    // UKMARS mazerunner-core compatible motion names.
    void move_ahead();
    void turn_left();
    void turn_right();
    void turn_back();
    void turn_smooth(int turn_id);
    void turn_IP180();
    void turn_IP90R();
    void turn_IP90L();

    // Arc turns (smooth turns with forward motion)
    void arcTurnLeft90();
    void arcTurnRight90();
    void arcTurnLeft45();
    void arcTurnRight45();

    // Execute command sequence like "L#F#R45#FH#GMF"
    void executeSequence(const std::string& sequence);

    void          setMovementStyle(MovementStyle style) { movement_style_ = style; }
    MovementStyle movementStyle() const { return movement_style_; }

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
    void        setUp(std::array<int, 2> start, std::vector<std::array<int, 2>> goals);
    void        printMaze();
    std::string mazeString();

    // Inject a motion-complete waiter (set by Cli on hardware; null in sim).
    // When set, every physical command waits until Core 1 completes the exact
    // command ID returned by CommandHub::send.
    void setMotionWaiter(MotionWaiter* waiter) { motion_waiter_ = waiter; }

    bool run_on_simulator = false;

  protected:
    void waitForMotion(uint16_t command_id)
    {
        if (motion_waiter_ != nullptr)
            motion_waiter_->waitForMotionComplete(command_id);
    }

  private:
    std::string simulatorResponse(const std::string& cmd);
    bool        simulatorBool(const std::string& cmd);
    std::string printMazeRow(int row);

    Mouse*        mouse_;
    char          phase_color_    = 'y';
    MotionWaiter* motion_waiter_  = nullptr;
    MovementStyle movement_style_ = MovementStyle::Stationary;
};

#endif
