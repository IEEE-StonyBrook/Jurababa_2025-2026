#ifndef APP_MOUSE_H
#define APP_MOUSE_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "config/motion.h"
#include "config/sensors.h"

class MazeMouse;
class Motion;
class Cell;
class BeaconIr;

/**
 * @brief High-level maze navigation Mouse
 *
 * Provides movement commands, wall sensing, and maze visualization. Bridges
 * between navigation algorithms and hardware control.
 *
 * Single-core UKMARS pattern: motion methods on hardware call directly into
 * `motion_->start_move()` / `motion_->start_turn()` and then busy-wait on
 * `motion_->move_finished()`/`turn_finished()` while a 500 Hz hardware timer
 * advances the controller in the background. Same shape as
 * mazerunner-core's `motion.move`.
 *
 * `wallLeft/Front/Right` are virtual so `FirmwareMouse` can read live ToF
 * distances on hardware while the simulator build keeps using bare `Mouse`.
 */
class Mouse
{
  public:
    enum class MovementStyle
    {
        Stationary,
        Smooth
    };

    enum class State
    {
        FRESH_START,
        SEARCHING,
        INPLACE_RUN,
        SMOOTH_RUN,
        FINISHED
    };

    explicit Mouse(MazeMouse* maze_mouse);
    virtual ~Mouse() = default;

    // Set the runtime motion target. Null on simulator builds; FirmwareMouse
    // requires a non-null Motion pointer for any motion to actually happen.
    void    set_motion(Motion* motion) { motion_ = motion; }
    Motion* motion() const { return motion_; }

    // Maze dimensions
    int mazeWidth();
    int mazeHeight();

    // Wall queries
    virtual bool wallLeft();
    virtual bool wallFront();
    virtual bool wallRight();
    bool         see_left_wall();
    bool         see_front_wall();
    bool         see_right_wall();
    virtual void serviceSensors();
    virtual void begin_motion_sequence();
    virtual void end_motion_sequence();
    void         begin_maze_heading_hold();
    void         end_maze_heading_hold();

    // UKMARS Mouse lifecycle/workflow names.
    void  init();
    void  set_heading(const std::string& heading);
    void  set_hand_start(bool hand_start) { m_handStart = hand_start; }
    bool  hand_start() const { return m_handStart; }
    State state() const { return state_; }

    // Movement commands
    void moveForwardHalf();
    bool move_mm(float distance_mm);
    bool move_physical(float distance_mm, float speed_mmps, float accel_mmps2);
    bool start_center();
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
    bool stopAtCentre();
    bool adjustPosition();
    bool wait_until_position(float position_mm);
    void update_map();
    bool search_to(const std::vector<std::array<int, 2>>& goals);
    bool search_maze();
    bool cheese_hunt(const std::array<int, 2>& start_cell, BeaconIr* beacon_ir);
    bool turn_to_face(const std::string& heading);
    void log_action_status(const std::string& action, Cell* cell, const std::string& position_text);
    void panic();
    bool run(float distance_mm);
    bool run_to(const std::vector<std::array<int, 2>>& goals, bool diagonalized);
    bool follow_to(const std::vector<std::array<int, 2>>& goals);
    bool wander_to(const std::vector<std::array<int, 2>>& goals);
    bool getRandomBool();
    std::string randomHeading();
    bool        test_SS90E();
    void        blink(int count);

    // UKMARS setup/report helpers. They intentionally use Jurababa ToF mm
    // readings and IMU yaw, but keep the mazerunner-core method names.
    void show_sensor_calibration();
    void print_wall_sensors();
    void report_profile();
    void front_sensor_track_header();
    void front_sensor_track();
    void report_sensor_track_header();
    void report_radial_track(bool use_raw = false);
    void conf_log_front_sensor();
    void conf_sensor_spin_calibrate();
    void conf_edge_detection();

    // Arc turns (smooth turns with forward motion)
    void arcTurnLeft90();
    void arcTurnRight90();
    void arcTurnLeft45();
    void arcTurnRight45();

    // Execute command sequence like "L#F#R45#FH#GMF"
    void executeSequence(const std::string& sequence);

    void          setMovementStyle(MovementStyle style) { movement_style_ = style; }
    MovementStyle movementStyle() const { return movement_style_; }

    // Forward cruise speed for straight-line cell traversal and post-turn
    // sense-window legs. Defaults to ROBOT_MAX_SEARCH_SPEED_MMPS so search
    // stages are unchanged. Stage 3+ explored-path runs override to a faster
    // value (typically ROBOT_MAX_FAST_SPEED_MMPS) and restore on exit.
    void  setCruiseSpeed(float mmps) { cruise_speed_mmps_ = mmps; }
    float cruiseSpeed() const { return cruise_speed_mmps_; }

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
    // UKMARS busy-wait pattern: spin on motion_->{move,turn}_finished() while
    // the 500 Hz timer ISR keeps Motion::update() advancing the controller.
    // Sensor service keeps ToF caches fresh during synchronous SEARCH waits.
    // 2 ms sleep matches mazerunner-core's `delay(2)` cadence.
    void waitForMotion();

    MazeMouse*    maze_mouse_        = nullptr;
    Motion*       motion_            = nullptr;
    HaltCheckFn   halt_check_        = nullptr;
    char          phase_color_       = 'y';
    MovementStyle movement_style_    = MovementStyle::Stationary;
    State         state_             = State::FRESH_START;
    bool          m_handStart        = false;
    float         cruise_speed_mmps_ = ROBOT_MAX_SEARCH_SPEED_MMPS;

    std::array<int, 2>              start_cell_ = {0, 0};
    std::vector<std::array<int, 2>> goal_cells_ = {};

  private:
    void        turn_to_cardinal_yaw(const std::string& target_heading);
    void        apply_maze_heading_hold();
    void        apply_maze_heading_hold_for_heading(const std::string& heading);
    void        suspend_maze_heading_hold();
    void        begin_search_front_latch();
    void        finish_search_front_latch();
    void        clear_search_front_latch();
    void        sample_search_front_latch();
    std::string simulatorResponse(const std::string& cmd);
    bool        simulatorBool(const std::string& cmd);
    std::string printMazeRow(int row);
    bool        maze_heading_hold_active_   = false;
    bool        search_front_latch_enabled_ = false;
    bool        search_front_wall_latched_  = false;
    float       search_front_latch_mm_      = TOF_OUT_OF_RANGE_MM;
};

#endif
