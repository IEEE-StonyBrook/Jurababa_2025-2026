#ifndef APP_API_H
#define APP_API_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class Mouse;
class Robot;

/**
 * @brief High-level maze navigation API
 *
 * Provides movement commands, wall sensing, and maze visualization. Bridges
 * between navigation algorithms and hardware control.
 *
 * Single-core UKMARS pattern: motion methods on hardware call directly into
 * `robot_->start_move()` / `robot_->start_turn()` and then busy-wait on
 * `robot_->move_finished()`/`turn_finished()` while a 500 Hz hardware timer
 * advances the controller in the background. Same shape as
 * mazerunner-core's `motion.move`.
 *
 * `wallLeft/Front/Right` are virtual so `FirmwareApi` can read live ToF
 * distances on hardware while the simulator build keeps using bare `API`.
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

    // Set the runtime motion target. Null on simulator builds; FirmwareApi
    // requires a non-null Robot pointer for any motion to actually happen.
    void   setRobot(Robot* robot) { robot_ = robot; }
    Robot* robot() const { return robot_; }

    // Maze dimensions
    int mazeWidth();
    int mazeHeight();

    // Wall queries
    virtual bool wallLeft();
    virtual bool wallFront();
    virtual bool wallRight();
    virtual void captureWallSample();
    virtual void setWallSample(int16_t left_mm, int16_t front_mm, int16_t right_mm);
    virtual void clearWallSample();
    virtual bool wallSample(int16_t& left_mm, int16_t& front_mm, int16_t& right_mm);
    virtual void serviceSensors();
    virtual void begin_motion_sequence();
    virtual void end_motion_sequence();

    // Movement commands
    void moveForwardHalf();
    bool move_mm(float distance_mm);
    bool move_physical(float distance_mm, float speed_mmps, float accel_mmps2);
    bool start_center();
    bool center_from_wall_check();
    bool search_start_from_wall_check();
    bool search_advance();
    void finish_search_move();
    void clear_search_move();
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

    // Halt hook used by long-running search loops to break early when the
    // operator hits HALT/X over Bluetooth. CLI installs a function pointer
    // that returns true when stop is requested. Null in simulator.
    using HaltCheckFn = bool (*)();
    void setHaltCheck(HaltCheckFn fn) { halt_check_ = fn; }
    bool haltRequested() const { return halt_check_ != nullptr && halt_check_(); }

    bool run_on_simulator = false;

  protected:
    // UKMARS busy-wait pattern: spin on robot_->{move,turn}_finished() while
    // the 500 Hz timer ISR keeps Robot::update() advancing the controller.
    // Sensor service keeps ToF caches fresh during synchronous SEARCH waits.
    // 2 ms sleep matches mazerunner-core's `delay(2)` cadence.
    void waitForMotion();

    Mouse*        mouse_          = nullptr;
    Robot*        robot_          = nullptr;
    HaltCheckFn   halt_check_     = nullptr;
    char          phase_color_    = 'y';
    MovementStyle movement_style_ = MovementStyle::Stationary;

  private:
    std::string simulatorResponse(const std::string& cmd);
    bool        simulatorBool(const std::string& cmd);
    std::string printMazeRow(int row);
};

#endif
