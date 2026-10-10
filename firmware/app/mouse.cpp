#include "app/mouse.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#ifndef SIMULATOR_BUILD
#include "pico/stdlib.h"

#include "app/leds.h"
#include "config/motion.h"
#include "control/motion.h"
#include "drivers/beacon_ir.h"
#endif
#include "common/log.h"
#include "common/tof_wall_utils.h"
#include "config/geometry.h"
#include "config/sensors.h"
#include "config/smooth_turn.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/a_star.h"
#include "navigation/diagonalizer.h"
#include "navigation/path_converter.h"

namespace
{
std::string uppercaseToken(std::string token)
{
    for (char& c : token)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return token;
}

bool parsePositiveInt(const std::string& text, int& value)
{
    if (text.empty())
        return false;

    int parsed = 0;
    for (char c : text)
    {
        if (!std::isdigit(static_cast<unsigned char>(c)))
            return false;
        parsed = parsed * 10 + (c - '0');
    }

    if (parsed <= 0)
        return false;

    value = parsed;
    return true;
}

[[maybe_unused]] std::string fixed1(float value)
{
#ifdef SIMULATOR_BUILD
    (void)value;
    return {};
#else
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
    return buffer;
#endif
}

[[maybe_unused]] std::string signedFixed1(float value)
{
#ifdef SIMULATOR_BUILD
    (void)value;
    return {};
#else
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%+.1f", static_cast<double>(value));
    return buffer;
#endif
}

#ifndef SIMULATOR_BUILD
float normalizeYawDelta(float degrees)
{
    while (degrees > 180.0f)
        degrees -= 360.0f;
    while (degrees <= -180.0f)
        degrees += 360.0f;
    return degrees;
}

float expectedYawForHeading(const std::string& heading)
{
    if (heading == "ne" || heading == "NE")
        return -45.0f;
    if (heading == "e" || heading == "E")
        return -90.0f;
    if (heading == "se" || heading == "SE")
        return -135.0f;
    if (heading == "s" || heading == "S")
        return 180.0f;
    if (heading == "sw" || heading == "SW")
        return 135.0f;
    if (heading == "w" || heading == "W")
        return 90.0f;
    if (heading == "nw" || heading == "NW")
        return 45.0f;
    return 0.0f;
}
#endif

#ifndef SEARCH_STEP_INFO_LOGGING_ENABLE
#define SEARCH_STEP_INFO_LOGGING_ENABLE 1
#endif

constexpr bool kSearchStepInfoLoggingEnabled = SEARCH_STEP_INFO_LOGGING_ENABLE != 0;

#ifndef SIMULATOR_BUILD
constexpr float    kMousePositionToleranceMm   = 0.125f;
constexpr uint32_t kMouseMotionWaitTimeoutMs   = 5000;
constexpr uint32_t kMousePositionWaitTimeoutMs = 3000;

bool timedOut(uint32_t start_ms, uint32_t timeout_ms)
{
    return timeout_ms > 0 &&
           static_cast<uint32_t>(to_ms_since_boot(get_absolute_time()) - start_ms) >= timeout_ms;
}
#endif

void logSearchStepInfo(const std::string& message)
{
    if (kSearchStepInfoLoggingEnabled)
        LOG_INFO(message);
}

std::string cardinalAfterHalfSteps(const std::string& heading, int half_steps_right)
{
    static constexpr const char* kCardinalHeadings[] = {"n", "e", "s", "w"};
    int                          index               = 0;
    if (heading == "e" || heading == "E")
        index = 1;
    else if (heading == "s" || heading == "S")
        index = 2;
    else if (heading == "w" || heading == "W")
        index = 3;

    const int cardinal_steps_right = half_steps_right / 2;
    index                          = (index + cardinal_steps_right) % 4;
    if (index < 0)
        index += 4;
    return kCardinalHeadings[index];
}

std::string headingUpper(const std::string& heading)
{
    std::string out = heading;
    for (char& c : out)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

const char* wallStateName(WallState state)
{
    switch (state)
    {
        case EXIT:
            return "EXIT";
        case WALL:
            return "WALL";
        case UNKNOWN:
            return "UNKNOWN";
        case VIRTUAL:
            return "VIRTUAL";
        default:
            return "?";
    }
}

std::string firstToken(const std::string& sequence, std::string* second = nullptr)
{
    std::istringstream ss(sequence);
    std::string        first;
    std::string        next;
    while (std::getline(ss, first, '#'))
    {
        if (!first.empty())
            break;
    }
    if (second != nullptr)
    {
        *second = "";
        while (std::getline(ss, next, '#'))
        {
            if (!next.empty())
            {
                *second = next;
                break;
            }
        }
    }
    return uppercaseToken(first);
}

std::string headingToNeighbor(Cell* from, Cell* to)
{
    if (from == nullptr || to == nullptr)
        return "";

    const int dx = to->x() - from->x();
    const int dy = to->y() - from->y();
    if (dx == 0 && dy == 1)
        return "n";
    if (dx == 1 && dy == 0)
        return "e";
    if (dx == 0 && dy == -1)
        return "s";
    if (dx == -1 && dy == 0)
        return "w";
    return "";
}

bool atAnyGoal(MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals)
{
    if (maze_mouse == nullptr)
        return false;
    Cell* current = maze_mouse->currentCell();
    if (current == nullptr)
        return false;
    for (const auto& goal : goals)
    {
        if (current->x() == goal[0] && current->y() == goal[1])
            return true;
    }
    return false;
}

std::vector<Cell*> pathToNearestUnexplored(MazeMouse* maze_mouse)
{
    Cell* start = maze_mouse != nullptr ? maze_mouse->currentCell() : nullptr;
    if (start == nullptr)
        return {};

    const int width  = maze_mouse->mazeWidth();
    const int height = maze_mouse->mazeHeight();

    std::vector<std::vector<bool>>  seen(width, std::vector<bool>(height, false));
    std::vector<std::vector<Cell*>> parent(width, std::vector<Cell*>(height, nullptr));
    std::queue<Cell*>               queue;

    seen[start->x()][start->y()] = true;
    queue.push(start);

    Cell* target = nullptr;
    while (!queue.empty() && target == nullptr)
    {
        Cell* current = queue.front();
        queue.pop();

        for (Cell* neighbor : maze_mouse->cellNeighbors(current, /*include_diagonal=*/false))
        {
            if (neighbor == nullptr || seen[neighbor->x()][neighbor->y()])
                continue;
            if (!maze_mouse->canMoveBetween(current, neighbor, /*diagonals=*/false))
                continue;

            seen[neighbor->x()][neighbor->y()]   = true;
            parent[neighbor->x()][neighbor->y()] = current;
            if (!neighbor->explored())
            {
                target = neighbor;
                break;
            }
            queue.push(neighbor);
        }
    }

    if (target == nullptr)
        return {};

    std::vector<Cell*> path;
    for (Cell* cell = target; cell != nullptr && cell != start; cell = parent[cell->x()][cell->y()])
        path.push_back(cell);
    std::reverse(path.begin(), path.end());
    return path;
}

class MouseMotionSequenceGuard
{
  public:
    explicit MouseMotionSequenceGuard(Mouse* mouse) : mouse_(mouse)
    {
        if (mouse_ != nullptr)
            mouse_->begin_motion_sequence();
    }

    ~MouseMotionSequenceGuard()
    {
        if (mouse_ != nullptr)
            mouse_->end_motion_sequence();
    }

    MouseMotionSequenceGuard(const MouseMotionSequenceGuard&)            = delete;
    MouseMotionSequenceGuard& operator=(const MouseMotionSequenceGuard&) = delete;

  private:
    Mouse* mouse_;
};

class MouseMazeHeadingHoldGuard
{
  public:
    explicit MouseMazeHeadingHoldGuard(Mouse* mouse) : mouse_(mouse)
    {
        if (mouse_ != nullptr)
            mouse_->begin_maze_heading_hold();
    }

    ~MouseMazeHeadingHoldGuard()
    {
        if (mouse_ != nullptr)
            mouse_->end_maze_heading_hold();
    }

    MouseMazeHeadingHoldGuard(const MouseMazeHeadingHoldGuard&)            = delete;
    MouseMazeHeadingHoldGuard& operator=(const MouseMazeHeadingHoldGuard&) = delete;

  private:
    Mouse* mouse_;
};
} // namespace

Mouse::Mouse(MazeMouse* maze_mouse) : run_on_simulator(false), maze_mouse_(maze_mouse)
{
    init();
}

void Mouse::init()
{
    m_handStart     = false;
    state_          = State::FRESH_START;
    movement_style_ = MovementStyle::Stationary;
    phase_color_    = 'y';
    if (maze_mouse_ != nullptr)
        maze_mouse_->reset(start_cell_, "n", goal_cells_);
}

void Mouse::set_heading(const std::string& heading)
{
    if (maze_mouse_ != nullptr)
        maze_mouse_->setDirection(heading);
}

template <typename Done> bool Mouse::pollUntil(Done done, uint32_t timeout_ms, const char* what)
{
#ifndef SIMULATOR_BUILD
    // UKMARS busy-wait: spin while the controller (running in the 500 Hz
    // hardware timer ISR) advances the profiles. 2 ms cadence matches
    // mazerunner-core's `delay(2)` wait loops. Halt or timeout stops the drive.
    if (motion_ == nullptr)
        return false;
    const uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (!done())
    {
        serviceSensors();
        sample_search_front_latch();
        if (haltRequested())
        {
            motion_->emergency_stop();
            return false;
        }
        if (timedOut(start_ms, timeout_ms))
        {
            LOG_ERROR("Mouse: " << what << " timeout position_mm=" << motion_->position()
                                << " velocity_mmps=" << motion_->velocity() << " angle_deg="
                                << motion_->angle() << " omega_degps=" << motion_->omega());
            motion_->emergency_stop();
            return false;
        }
        sleep_ms(2);
    }
    return !haltRequested();
#else
    (void)done;
    (void)timeout_ms;
    (void)what;
    return true;
#endif
}

bool Mouse::waitForMotion()
{
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    return pollUntil([this] { return motion_->move_finished() && motion_->turn_finished(); },
                     kMouseMotionWaitTimeoutMs, "waitForMotion");
#else
    return true;
#endif
}

void Mouse::begin_search_front_latch()
{
    search_front_wall_latched_  = false;
    search_front_latch_mm_      = TOF_OUT_OF_RANGE_MM;
    search_front_latch_enabled_ = !run_on_simulator && motion_ != nullptr;
}

void Mouse::finish_search_front_latch()
{
    search_front_latch_enabled_ = false;
}

void Mouse::clear_search_front_latch()
{
    search_front_latch_enabled_ = false;
    search_front_wall_latched_  = false;
    search_front_latch_mm_      = TOF_OUT_OF_RANGE_MM;
}

void Mouse::sample_search_front_latch()
{
#ifndef SIMULATOR_BUILD
    if (!search_front_latch_enabled_ || run_on_simulator || motion_ == nullptr)
        return;

    const float front_mm = motion_->frontDistance();
    if (!tof_wall::wallFront(front_mm))
        return;

    search_front_wall_latched_ = true;
    if (front_mm < search_front_latch_mm_)
        search_front_latch_mm_ = front_mm;
#endif
}

int Mouse::mazeWidth()
{
    return maze_mouse_->mazeWidth();
}

int Mouse::mazeHeight()
{
    return maze_mouse_->mazeHeight();
}

bool Mouse::wallLeft()
{
    return run_on_simulator ? simulatorBool("wallLeft") : false;
}

bool Mouse::wallFront()
{
    return run_on_simulator ? simulatorBool("wallFront") : false;
}

bool Mouse::wallRight()
{
    return run_on_simulator ? simulatorBool("wallRight") : false;
}

bool Mouse::see_left_wall()
{
    return wallLeft();
}

bool Mouse::see_front_wall()
{
    return wallFront();
}

bool Mouse::see_right_wall()
{
    return wallRight();
}

void Mouse::serviceSensors()
{
}

void Mouse::begin_motion_sequence()
{
    clear_search_front_latch();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->begin_motion_sequence();
#endif
}

void Mouse::end_motion_sequence()
{
    clear_search_front_latch();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->end_motion_sequence();
#endif
}

void Mouse::begin_maze_heading_hold()
{
    maze_heading_hold_active_ = true;
    apply_maze_heading_hold();
}

void Mouse::end_maze_heading_hold()
{
    maze_heading_hold_active_ = false;
    suspend_maze_heading_hold();
}

void Mouse::apply_maze_heading_hold()
{
    if (!maze_heading_hold_active_ || run_on_simulator || maze_mouse_ == nullptr)
        return;
    apply_maze_heading_hold_for_heading(maze_mouse_->currentDirection());
}

void Mouse::apply_maze_heading_hold_for_heading(const std::string& heading)
{
    if (!maze_heading_hold_active_ || run_on_simulator)
        return;
#ifndef SIMULATOR_BUILD
    if (motion_ != nullptr)
        motion_->set_heading_hold(expectedYawForHeading(heading));
#else
    (void)heading;
#endif
}

void Mouse::suspend_maze_heading_hold()
{
    if (run_on_simulator)
        return;
#ifndef SIMULATOR_BUILD
    if (motion_ != nullptr)
        motion_->clear_heading_hold();
#endif
}

void Mouse::moveForwardHalf()
{
    if (run_on_simulator)
        simulatorResponse("moveForwardHalf");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        apply_maze_heading_hold();
        motion_->move(HALF_CELL_MM, cruise_speed_mmps_, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        if (!waitForMotion())
            return;
    }
#endif
    // Half-cell physical moves do not advance the logical maze cell. Diagonal
    // sequences use GMF/GFM when the virtual mouse should move to the next cell.
}

bool Mouse::move_mm(float distance_mm)
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    LOG_INFO("MOTION move_mm: distance_mm=" + fixed1(distance_mm) + " speed_mmps=" +
             fixed1(cruise_speed_mmps_) + " accel_mmps2=" + fixed1(ROBOT_BASE_ACCEL_MMPS2));
    apply_maze_heading_hold();
    motion_->move(distance_mm, cruise_speed_mmps_, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    return waitForMotion() && !haltRequested();
#else
    (void)distance_mm;
    return true;
#endif
}

bool Mouse::move_physical(float distance_mm, float speed_mmps, float accel_mmps2)
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    LOG_INFO("MOTION move_physical: distance_mm=" + fixed1(distance_mm) +
             " speed_mmps=" + fixed1(speed_mmps) + " accel_mmps2=" + fixed1(accel_mmps2));
    apply_maze_heading_hold();
    motion_->move(distance_mm, speed_mmps, 0.0f, accel_mmps2);
    return waitForMotion() && !haltRequested();
#else
    (void)distance_mm;
    (void)speed_mmps;
    (void)accel_mmps2;
    return true;
#endif
}

bool Mouse::start_center()
{
    return move_mm(START_CENTER_DISTANCE_MM);
}

void Mouse::moveForward()
{
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        apply_maze_heading_hold();
        motion_->move(CELL_SIZE_MM, cruise_speed_mmps_, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        if (!waitForMotion())
            return;
    }
#endif
    maze_mouse_->moveForward(1);
}

void Mouse::moveForward(int steps)
{
    // For simulator, call single-step version in a loop (mms doesn't support moveForwardN)
    if (run_on_simulator)
    {
        for (int i = 0; i < steps; i++)
        {
            simulatorResponse("moveForward");
            maze_mouse_->moveForward(1);
        }
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ != nullptr)
    {
        apply_maze_heading_hold();
        motion_->move(steps * CELL_SIZE_MM, cruise_speed_mmps_, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        if (!waitForMotion())
            return;
    }
#endif
    maze_mouse_->moveForward(steps);
}

void Mouse::ghostMoveForward(int steps)
{
    // Ghost move only updates internal mouse position
    maze_mouse_->moveForward(steps);
}

void Mouse::turnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft45");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        suspend_maze_heading_hold();
        if (!motion_->spin_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS,
                                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2) ||
            !waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(-1);
}

void Mouse::turnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        LOG_INFO(
            "MOTION turnLeft90: angle_deg=90 omega_degps=" + fixed1(ROBOT_MAX_TURN_SPEED_DEGPS) +
            " alpha_degps2=" + fixed1(ROBOT_BASE_ANGULAR_ACCEL_DEGPS2));
        suspend_maze_heading_hold();
        if (!motion_->spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS,
                                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2) ||
            !waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(-2);
}

void Mouse::turnRight45()
{
    if (run_on_simulator)
        simulatorResponse("turnRight45");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        suspend_maze_heading_hold();
        if (!motion_->spin_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS,
                                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2) ||
            !waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(1);
}

void Mouse::turnRight90()
{
    if (run_on_simulator)
        simulatorResponse("turnRight");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        LOG_INFO(
            "MOTION turnRight90: angle_deg=-90 omega_degps=" + fixed1(ROBOT_MAX_TURN_SPEED_DEGPS) +
            " alpha_degps2=" + fixed1(ROBOT_BASE_ANGULAR_ACCEL_DEGPS2));
        suspend_maze_heading_hold();
        if (!motion_->spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS,
                                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2) ||
            !waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(2);
}

void Mouse::turn(int degrees)
{
    if (run_on_simulator)
        simulatorResponse("turn" + std::to_string(degrees));
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        suspend_maze_heading_hold();
        if (!motion_->spin_turn(static_cast<float>(degrees), ROBOT_MAX_TURN_SPEED_DEGPS,
                                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2) ||
            !waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(degrees / 45);
}

void Mouse::move_ahead()
{
    if (run_on_simulator)
    {
        simulatorResponse("moveForward");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return;
    LOG_INFO("move_ahead: adjust_forward_position=-" + fixed1(CELL_SIZE_MM) +
             " wait_until_position=" + fixed1(SENSING_POSITION_MM));
    apply_maze_heading_hold();
    motion_->resetWallSteeringStats();
    begin_search_front_latch();
    motion_->adjust_forward_position(-CELL_SIZE_MM);
    if (!wait_until_position(SENSING_POSITION_MM))
    {
        clear_search_front_latch();
        return;
    }
    finish_search_front_latch();
    // No set_position(SENSING_POSITION_MM) here, as in mazerunner-core: the
    // poll lands a little past 170 mm, and resetting would erase that distance
    // every cell, so the robot drifts further than it thinks over a straight.
#endif
}

bool Mouse::turn_to_cardinal_yaw(const std::string& target_heading)
{
    if (run_on_simulator)
    {
        if (maze_mouse_ == nullptr)
            return false;
        const int current_index = maze_mouse_->findDirectionIndex(maze_mouse_->currentDirection());
        const int target_index  = maze_mouse_->findDirectionIndex(target_heading);
        if (current_index < 0 || target_index < 0)
            return false;

        const int half_steps_right = (target_index - current_index + 8) % 8;
        if (half_steps_right == 1)
            simulatorResponse("turnRight45");
        else if (half_steps_right == 2)
            simulatorResponse("turnRight");
        else if (half_steps_right == 3)
        {
            simulatorResponse("turnRight");
            simulatorResponse("turnRight45");
        }
        else if (half_steps_right == 4)
        {
            simulatorResponse("turnRight");
            simulatorResponse("turnRight");
        }
        else if (half_steps_right == 5)
        {
            simulatorResponse("turnLeft");
            simulatorResponse("turnLeft45");
        }
        else if (half_steps_right == 6)
            simulatorResponse("turnLeft");
        else if (half_steps_right == 7)
            simulatorResponse("turnLeft45");
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;

    const float target_yaw_deg  = expectedYawForHeading(target_heading);
    const float current_yaw_deg = motion_->yaw_deg();
    const float delta_deg       = normalizeYawDelta(target_yaw_deg - current_yaw_deg);
    LOG_INFO("SEARCH TURN target=" + headingUpper(target_heading) +
             " yaw=" + fixed1(current_yaw_deg) + " target_yaw=" + fixed1(target_yaw_deg) +
             " delta=" + signedFixed1(delta_deg));
    suspend_maze_heading_hold();
    if (!motion_->spin_turn(delta_deg, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2))
        return false;
    return waitForMotion();
#else
    (void)target_heading;
    return true;
#endif
}

void Mouse::turn_left()
{
    turn_side(/*turn_left=*/true);
}

void Mouse::turn_right()
{
    turn_side(/*turn_left=*/false);
}

void Mouse::turn_side(bool turn_left)
{
    const int half_steps = turn_left ? -2 : 2;
    if (movement_style_ == MovementStyle::Smooth)
    {
        smooth_turn_step(turn_left);
        maze_mouse_->turn45Steps(half_steps);
        // Hold the new heading so a turn straight after this one can aim at it.
        apply_maze_heading_hold();
        return;
    }
    stationary_turn(turn_left ? maze_mouse_->directionLeft() : maze_mouse_->directionRight(),
                    half_steps);
}

void Mouse::turn_back()
{
    stationary_turn(cardinalAfterHalfSteps(maze_mouse_->currentDirection(), 4), 4);
}

void Mouse::stationary_turn(const std::string& target_heading, int half_steps)
{
    // mazerunner-core turn_back(), used for every stop-and-spin turn: stop at
    // the centre, square up on the front wall, spin, then run back out to the
    // sensing point at search speed.
    if (!stopAtCentre() || !adjustPosition())
        return;
    // At a dead end, back off so the spin has room; that puts the axle the
    // back-off past the centre on the new heading.
    float start_mm = HALF_CELL_MM;
#ifndef SIMULATOR_BUILD
    if (half_steps == 4 && !run_on_simulator && motion_ != nullptr &&
        tof_wall::wallFront(motion_->frontDistance()))
    {
        motion_->move(-DEAD_END_SPIN_BACKOFF_MM, ROBOT_MAX_SEARCH_SPEED_MMPS / 4.0f, 0.0f,
                      ROBOT_SEARCH_ACCEL_MMPS2 / 2.0f);
        if (!waitForMotion())
            return;
        start_mm += DEAD_END_SPIN_BACKOFF_MM;
    }
#endif
    if (!turn_to_cardinal_yaw(target_heading))
        return;
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_position(start_mm);
        apply_maze_heading_hold_for_heading(target_heading);
        motion_->resetWallSteeringStats();
        begin_search_front_latch();
        motion_->move(SENSING_POSITION_MM - start_mm, cruise_speed_mmps_, cruise_speed_mmps_,
                      ROBOT_SEARCH_ACCEL_MMPS2);
        if (!waitForMotion())
        {
            clear_search_front_latch();
            return;
        }
        finish_search_front_latch();
        motion_->set_position(SENSING_POSITION_MM);
    }
#endif
    maze_mouse_->turn45Steps(half_steps);
}

bool Mouse::smooth_turn(bool turn_left, float centre_ahead_mm, float end_after_centre_mm)
{
    // mazerunner-core Mouse::turn_smooth(), shared by search, SMOOTH RUN and
    // PATH ... SMOOTH so a turn tuned on one is tuned on all. centre_ahead_mm
    // is the distance from here to the turn cell centre; the robot finishes
    // end_after_centre_mm past it on the new heading, still at turn speed.
    if (run_on_simulator)
    {
        simulatorResponse(turn_left ? "arcTurnLeft90" : "arcTurnRight90");
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
    // Aim the arc at the heading the straight was holding, as the spin turns to
    // a cardinal yaw: an arc only turns a fixed angle, so a back-to-back turn
    // (no straight to correct on) would carry the last arc's error forward.
    const bool  aim_arc     = motion_->headingHoldActive();
    const float aim_yaw_deg = motion_->headingHoldTargetYawDeg();
    suspend_maze_heading_hold();

    // Front-sensor trigger: with a wall ahead, start the arc once the front
    // ToF reads the trigger distance even if odometry says "not yet". ToF
    // distance shrinks as the wall nears, the reverse of mazerunner's sensor
    // sum. Side walls can make the front ToF read short, so each one seen
    // lowers the trigger (mazerunner EXTRA_WALL_ADJUST).
    const Motion::SmoothTurnTuning& tune       = motion_->smoothTurnTuning();
    float                           trigger_mm = tune.side(turn_left).front_trigger_mm;
    if (trigger_mm > 0.0f)
        trigger_mm -=
            tune.extra_wall_adjust_mm * ((wallLeft() ? 1.0f : 0.0f) + (wallRight() ? 1.0f : 0.0f));

    // mazerunner: while (position < turn_point) { if (front_sum > trigger) break; }
    const float lead_mm   = motion_->start_smooth_turn_entry(turn_left, centre_ahead_mm);
    bool        by_sensor = false;
    if (!pollUntil(
            [&]
            {
                const float front_mm = motion_->frontDistance();
                by_sensor =
                    trigger_mm > 0.0f && tof_wall::wallFront(front_mm) && front_mm <= trigger_mm;
                return by_sensor || motion_->position() >= lead_mm;
            },
            kMouseMotionWaitTimeoutMs, "smooth_turn entry"))
        return false;
    // Read late, after the entry leg: the BNO085 lags ~40-60 ms behind an arc.
    // Clamped so a bad reading can't turn a 90 into something else.
    const float aim_error_deg =
        aim_arc ? std::clamp(normalizeYawDelta(aim_yaw_deg - motion_->yaw_deg()),
                             -SMOOTH_TURN_MAX_AIM_DEG, SMOOTH_TURN_MAX_AIM_DEG)
                : 0.0f;
    motion_->start_smooth_turn_arc(turn_left, aim_error_deg);
    LOG_INFO("SMOOTH TURN " + std::string(turn_left ? "L" : "R") + " aim_deg=" +
             signedFixed1(aim_error_deg) + " trigger=" + std::string(by_sensor ? "s" : "d") +
             " front_mm=" + fixed1(motion_->frontDistance()) + " trigger_mm=" + fixed1(trigger_mm) +
             " position_mm=" + fixed1(motion_->position()));
    if (!waitForMotion())
        return false;

    // Exit leg at turn speed. mazerunner's exit move asks for SEARCH_SPEED but
    // its Profile clamps final speed to the top (current turn) speed, so the
    // robot stays at turn speed afterwards and every later turn starts at it.
    const float speed_mmps = tune.speed_mmps;
    motion_->move(motion_->smoothTurnExitMm(turn_left, end_after_centre_mm), speed_mmps, speed_mmps,
                  ROBOT_SEARCH_ACCEL_MMPS2);
    return waitForMotion();
#else
    (void)centre_ahead_mm;
    (void)end_after_centre_mm;
    return true;
#endif
}

bool Mouse::smooth_turn_step(bool turn_left)
{
    // One search smooth turn, sensing point to sensing point. Position is in
    // the cell frame, so the turn cell centre is at CELL_SIZE_MM + HALF_CELL_MM
    // however long the planner took to get here.
    float centre_ahead_mm = 0.0f;
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        centre_ahead_mm = CELL_SIZE_MM + HALF_CELL_MM - motion_->position();
#endif
    if (!smooth_turn(turn_left, centre_ahead_mm, SENSING_POSITION_MM - HALF_CELL_MM))
        return false;
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_position(SENSING_POSITION_MM);
        // mazerunner's search loop re-enables steering at every sensing point.
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif
    return true;
}

bool Mouse::turn_IP180()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        simulatorResponse("turnRight");
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    suspend_maze_heading_hold();
    return motion_->turn_IP180() && waitForMotion();
#else
    return true;
#endif
}

bool Mouse::turn_IP90R()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    suspend_maze_heading_hold();
    return motion_->turn_IP90R() && waitForMotion();
#else
    return true;
#endif
}

bool Mouse::turn_IP90L()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnLeft");
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    suspend_maze_heading_hold();
    return motion_->turn_IP90L() && waitForMotion();
#else
    return true;
#endif
}

bool Mouse::wait_until_position(float position_mm)
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    return pollUntil([&] { return motion_->position() >= position_mm - kMousePositionToleranceMm; },
                     kMousePositionWaitTimeoutMs, "wait_until_position");
#else
    (void)position_mm;
    return true;
#endif
}

bool Mouse::frontWallInThisCell(float centre_ahead_mm)
{
#ifndef SIMULATOR_BUILD
    // A wall in this cell reads about FRONT_REFERENCE_MM + centre_ahead_mm; one
    // a cell further reads ~CELL_SIZE_MM more. Cut halfway, so a far wall that
    // still passes wallFront()'s 250 mm mapping threshold is not mistaken for
    // this cell's (seen as a 20 mm "correction" toward a wall at 175 mm).
    serviceSensors();
    return wallFront() &&
           motion_->frontDistance() < FRONT_REFERENCE_MM + centre_ahead_mm + HALF_CELL_MM;
#else
    (void)centre_ahead_mm;
    return false;
#endif
}

bool Mouse::stopAtCentre()
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    const float distance_mm = CELL_SIZE_MM + HALF_CELL_MM - motion_->position();
    LOG_INFO("stopAtCentre: position_mm=" + fixed1(motion_->position()) +
             " distance_mm=" + fixed1(distance_mm));
    apply_maze_heading_hold();

    if (!frontWallInThisCell(distance_mm))
    {
        motion_->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_SEARCH_ACCEL_MMPS2);
        if (!waitForMotion())
            return false;
        motion_->set_position(HALF_CELL_MM);
        return true;
    }

    // mazerunner-core stopAtCentre(): with a wall ahead, arrive still creeping
    // at STOP_AT_CENTRE_FRONT_SPEED_MMPS and stop on the front sensor, so the
    // stop is where the wall says the centre is, not where odometry thinks it
    // is. 500 ms from the start of the approach, as in mazerunner.
    motion_->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, STOP_AT_CENTRE_FRONT_SPEED_MMPS,
                  ROBOT_SEARCH_ACCEL_MMPS2);
    const uint32_t start_ms  = to_ms_since_boot(get_absolute_time());
    bool           by_sensor = false;
    if (!pollUntil(
            [&]
            {
                const float front_mm = motion_->frontDistance();
                by_sensor = tof_wall::wallFront(front_mm) && front_mm <= FRONT_REFERENCE_MM;
                return by_sensor || timedOut(start_ms, STOP_AT_CENTRE_FRONT_TIMEOUT_MS);
            },
            kMouseMotionWaitTimeoutMs, "stopAtCentre"))
        return false;
    if (!motion_->stop_forward(ROBOT_SEARCH_ACCEL_MMPS2, STOP_AT_CENTRE_PROFILE_TIMEOUT_MS))
        return false;
    LOG_INFO("stopAtCentre: reason=" + std::string(by_sensor ? "sensor" : "timeout") +
             " front_mm=" + fixed1(motion_->frontDistance()));
    motion_->set_position(HALF_CELL_MM);
#endif
    return true;
}

bool Mouse::adjustPosition()
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;

    if (!frontWallInThisCell(/*centre_ahead_mm=*/0.0f))
        return true;
    const float front_mm      = motion_->frontDistance();
    float       correction_mm = front_mm - FRONT_REFERENCE_MM;
    if (std::fabs(correction_mm) <= FRONT_CORRECTION_TOLERANCE_MM)
        return true;

    if (correction_mm > FRONT_CORRECTION_STEP_MM)
        correction_mm = FRONT_CORRECTION_STEP_MM;
    if (correction_mm < -FRONT_CORRECTION_STEP_MM)
        correction_mm = -FRONT_CORRECTION_STEP_MM;

    LOG_INFO("adjustPosition: front_mm=" + fixed1(front_mm) +
             " correction_mm=" + fixed1(correction_mm));
    apply_maze_heading_hold();
    motion_->move(correction_mm, FRONT_CORRECTION_SPEED_MMPS, 0.0f, FRONT_CORRECTION_ACCEL_MMPS2);
    if (!waitForMotion())
        return false;
    motion_->set_position(HALF_CELL_MM);
    return !haltRequested();
#else
    return true;
#endif
}

void Mouse::update_map()
{
    serviceSensors();
    Cell* cell = maze_mouse_ != nullptr ? maze_mouse_->currentCell() : nullptr;
    if (cell == nullptr)
        return;

    const int         x     = cell->x();
    const int         y     = cell->y();
    const std::string front = maze_mouse_->directionAsString(maze_mouse_->currentDirectionArray());
    const std::string left  = maze_mouse_->directionLeft();
    const std::string right = maze_mouse_->directionRight();
    const bool        left_wall          = wallLeft();
    const bool        instant_front_wall = wallFront();
    const bool        front_wall         = instant_front_wall || search_front_wall_latched_;
    const bool        right_wall         = wallRight();

    auto log_known_conflict = [&](char direction, WallState observed_state)
    {
        const WallState known_state = cell->wallState(direction);
        if (known_state == UNKNOWN || known_state == observed_state)
            return;
        LOG_ERROR("SEARCH MAP ignored conflicting wall observation cell=(" + std::to_string(x) +
                  "," + std::to_string(y) + ") dir=" + std::string(1, direction) + " known=" +
                  wallStateName(known_state) + " observed=" + wallStateName(observed_state));
    };

    log_known_conflict(front[0], front_wall ? WALL : EXIT);
    log_known_conflict(left[0], left_wall ? WALL : EXIT);
    log_known_conflict(right[0], right_wall ? WALL : EXIT);

    cell->updateWallState(front[0], front_wall ? WALL : EXIT);
    cell->updateWallState(left[0], left_wall ? WALL : EXIT);
    cell->updateWallState(right[0], right_wall ? WALL : EXIT);

    std::ostringstream map_log;
    map_log << "SEARCH MAP cell=(" << x << "," << y
            << ") hdg=" << headingUpper(maze_mouse_->currentDirection())
            << " walls=" << (left_wall ? "L" : "-") << (front_wall ? "F" : "-")
            << (right_wall ? "R" : "-");
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        map_log << " tofRaw[L=" << fixed1(motion_->leftRawDistance())
                << " F=" << fixed1(motion_->frontRawDistance())
                << " R=" << fixed1(motion_->rightRawDistance())
                << "] tofFilt[L=" << fixed1(motion_->leftDistance())
                << " F=" << fixed1(motion_->frontDistance())
                << " R=" << fixed1(motion_->rightDistance()) << "]"
                << " instantF=" << (instant_front_wall ? 1 : 0)
                << " latchF=" << (search_front_wall_latched_ ? 1 : 0);
        if (search_front_wall_latched_)
            map_log << " latch_mm=" << fixed1(search_front_latch_mm_);
        if (instant_front_wall)
        {
            // Odometry check: the wall seen is the far side of the cell being
            // entered. err (true mm) > 0 means the robot is short of where it thinks.
            const float wall_ahead_mm =
                2.0f * CELL_SIZE_MM - WALL_THICKNESS_MM / 2.0f - motion_->position();
            const float expected_mm = FRONT_TOF_GAIN * wall_ahead_mm - FRONT_TOF_OFFSET_MM +
                                      motion_->velocity() * FRONT_TOF_LAG_S;
            map_log << " fwd[exp=" << fixed1(expected_mm) << " err="
                    << signedFixed1((motion_->frontDistance() - expected_mm) / FRONT_TOF_GAIN)
                    << " v=" << fixed1(motion_->velocity()) << "]";
        }
        map_log << "]";
    }
#endif
    logSearchStepInfo(map_log.str());

    if (run_on_simulator)
    {
        if (front_wall)
            setWall(x, y, front);
        if (left_wall)
            setWall(x, y, left);
        if (right_wall)
            setWall(x, y, right);
    }

    cell->markExplored();
    clear_search_front_latch();
}

bool Mouse::search_to(const std::vector<std::array<int, 2>>& goals)
{
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard  motion_sequence(this);
    MouseMazeHeadingHoldGuard heading_hold(this);
    state_ = movement_style_ == MovementStyle::Smooth ? State::SMOOTH_RUN : State::SEARCHING;
    maze_mouse_->setMazeMask(MASK_OPEN);

    if (!m_handStart)
    {
        Cell* current = maze_mouse_->currentCell();
        if (current == nullptr)
            return false;

        for (const auto& goal : goals)
        {
            if (current->x() == goal[0] && current->y() == goal[1])
                return true;
        }

        AStar              initial_a_star(maze_mouse_);
        std::vector<Cell*> initial_path = initial_a_star.cellPath(goals, /*diagonals=*/false,
                                                                  /*pass_goals=*/true);
        if (initial_path.empty())
        {
            LOG_ERROR("search_to: no initial route from (" + std::to_string(current->x()) + "," +
                      std::to_string(current->y()) + ")");
            return false;
        }

        const std::string initial_heading = headingToNeighbor(current, initial_path.front());
        if (!initial_heading.empty() && !turn_to_face(initial_heading))
            return false;
    }

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator)
    {
        if (motion_ == nullptr)
            return false;
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        motion_->resetWallSteeringStats();
        begin_search_front_latch();
        float sensing_wait_position_mm = SENSING_POSITION_MM - HALF_CELL_MM;
        if (m_handStart)
        {
            sensing_wait_position_mm =
                START_CENTER_DISTANCE_MM + (SENSING_POSITION_MM - HALF_CELL_MM);
            LOG_INFO(
                "search_to: hand_start chained_distance_mm=" + fixed1(sensing_wait_position_mm) +
                " start_center_mm=" + fixed1(START_CENTER_DISTANCE_MM) +
                " sensing_position_mm=" + fixed1(SENSING_POSITION_MM));
            apply_maze_heading_hold();
            motion_->move(sensing_wait_position_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
            m_handStart = false;
        }
        else
        {
            // mazerunner-core search_to(): back up to the wall behind at a
            // quarter of search speed so the leg starts squared to it and at a
            // known distance, then drive off as from a hand start. As in
            // mazerunner, nothing checks that a wall is there; without one the
            // robot just reverses and returns, unaligned.
            LOG_INFO("search_to: back_up_mm=" + fixed1(START_CENTER_DISTANCE_MM));
            motion_->move(-START_CENTER_DISTANCE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS / 4.0f, 0.0f,
                          ROBOT_SEARCH_ACCEL_MMPS2 / 2.0f);
            if (!waitForMotion())
            {
                clear_search_front_latch();
                return false;
            }
            sensing_wait_position_mm =
                START_CENTER_DISTANCE_MM + (SENSING_POSITION_MM - HALF_CELL_MM);
            apply_maze_heading_hold();
            motion_->move(sensing_wait_position_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
        }
        if (!wait_until_position(sensing_wait_position_mm))
        {
            clear_search_front_latch();
            return false;
        }
        finish_search_front_latch();
        // Re-label into the cell frame but keep the distance already past the
        // wait point (see move_ahead()).
        motion_->set_position(SENSING_POSITION_MM +
                              (motion_->position() - sensing_wait_position_mm));
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif
    if (run_on_simulator)
        simulatorResponse("moveForward");

    // A* targets: the goals until the goal area is entered, then its
    // unexplored cells, then a goal cell with a wall ahead to stop in.
    std::vector<std::array<int, 2>> targets            = goals;
    int                             goal_extra_steps   = 0;
    constexpr int                   kMaxGoalExtraSteps = 2;

    int step = 1;
    while (true)
    {
        maze_mouse_->moveForward(1);
        Cell* current = maze_mouse_->currentCell();
        if (current == nullptr)
            return false;

        update_map();

        bool reached_goal = false;
        for (const auto& goal : goals)
        {
            if (current->x() == goal[0] && current->y() == goal[1])
            {
                reached_goal = true;
                break;
            }
        }
        if (reached_goal)
        {
            // Explore every goal cell, as other mice do, then stop in one with
            // a wall ahead: stopAtCentre() and adjustPosition() square on it,
            // and the next leg backs into it.
            std::vector<std::array<int, 2>> unexplored;
            std::vector<std::array<int, 2>> others;
            for (const auto& goal : goals)
            {
                if (current->x() == goal[0] && current->y() == goal[1])
                    continue;
                others.push_back(goal);
                Cell* goal_cell = maze_mouse_->cellAt(goal[0], goal[1]);
                if (goal_cell != nullptr && !goal_cell->explored())
                    unexplored.push_back(goal);
            }
            const char front_dir =
                maze_mouse_->directionAsString(maze_mouse_->currentDirectionArray())[0];
            const bool wall_ahead = current->wallState(front_dir) == WALL;
            targets.clear();
            if (!unexplored.empty())
                targets = unexplored;
            else if (!wall_ahead && !others.empty() && goal_extra_steps < kMaxGoalExtraSteps)
            {
                targets = others;
                ++goal_extra_steps;
            }
            if (!targets.empty())
            {
                AStar              goal_a_star(maze_mouse_);
                std::vector<Cell*> goal_path = goal_a_star.cellPath(targets, /*diagonals=*/false,
                                                                    /*pass_goals=*/true);
                if (goal_path.empty())
                    targets.clear(); // walled off: stop here
            }
        }
        if (reached_goal && targets.empty())
        {
            LOG_INFO("search_to: goal reached at (" + std::to_string(current->x()) + "," +
                     std::to_string(current->y()) + ")");
            if (!stopAtCentre())
                return false;
            if (!adjustPosition())
                return false;
            state_ = State::FINISHED;
            return true;
        }

        AStar a_star(maze_mouse_);
        // Way home: take an unexplored route unless it is ~1.5x longer.
        if (goals.size() == 1 && goals[0] == start_cell_)
            a_star.set_explored_cell_cost(0.5f);
        std::vector<Cell*> cell_path = a_star.cellPath(targets, /*diagonals=*/false,
                                                       /*pass_goals=*/true);
        if (cell_path.empty())
        {
            LOG_ERROR("search_to: no path found from (" + std::to_string(current->x()) + "," +
                      std::to_string(current->y()) + ")");
            // The chained move is still rolling at search speed; returning without
            // a stop drove the robot into the wall ahead.
            stopAtCentre();
            return false;
        }

        std::string lfr =
            PathConverter::buildLFR(current, maze_mouse_->currentDirectionArray(), cell_path);
        std::string second;
        std::string action = firstToken(lfr, &second);
#ifndef SIMULATOR_BUILD
        const std::string position_text =
            motion_ != nullptr ? fixed1(motion_->position()) : std::string("NA");
#else
        const std::string position_text = "SIM";
#endif
        logSearchStepInfo("search_to step=" + std::to_string(step) + " cell=(" +
                          std::to_string(current->x()) + "," + std::to_string(current->y()) +
                          ") heading=" + headingUpper(maze_mouse_->currentDirection()) +
                          " entered=1 lfr=" + lfr + " action=" + action +
                          " position_mm=" + position_text);
        log_action_status(action, current, position_text);

        const int         from_x       = current->x();
        const int         from_y       = current->y();
        const std::string from_heading = headingUpper(maze_mouse_->currentDirection());
#ifndef SIMULATOR_BUILD
        const float yaw_before = motion_ != nullptr ? motion_->yaw_deg() : 0.0f;
        const float pos_before = motion_ != nullptr ? motion_->position() : 0.0f;
#endif

        if (action == "F")
            move_ahead();
        else if ((action == "L" && second == "L") || (action == "R" && second == "R") ||
                 action == "B")
            turn_back();
        else if (action == "L")
            turn_left();
        else if (action == "R")
            turn_right();
        else
        {
            LOG_ERROR("search_to: unsupported LFR action: " + action);
            return false;
        }

        Cell*              after_cell    = maze_mouse_->currentCell();
        const std::string  after_heading = headingUpper(maze_mouse_->currentDirection());
        std::ostringstream action_log;
        action_log << "SEARCH ACT " << action << " from=(" << from_x << "," << from_y << ","
                   << from_heading << ") to=("
                   << (after_cell != nullptr ? std::to_string(after_cell->x()) : std::string("NA"))
                   << ","
                   << (after_cell != nullptr ? std::to_string(after_cell->y()) : std::string("NA"))
                   << "," << after_heading << ")";
#ifndef SIMULATOR_BUILD
        if (motion_ != nullptr)
        {
            const float yaw_after       = motion_->yaw_deg();
            const float pos_after       = motion_->position();
            const float yaw_delta       = normalizeYawDelta(yaw_after - yaw_before);
            const float expected_yaw    = expectedYawForHeading(maze_mouse_->currentDirection());
            const float final_yaw_error = normalizeYawDelta(expected_yaw - yaw_after);
            const tof_wall::WallState wall_state        = motion_->wallSteeringState();
            const float               tof_steer_degps   = motion_->wallSteeringAdjustmentDegps();
            const float               imu_steer_degps   = motion_->headingHoldAdjustmentDegps();
            const float               total_steer_degps = tof_steer_degps + imu_steer_degps;
            const Motion::WallSteeringStats steer_stats = motion_->wallSteeringStats();
            action_log << " yaw=" << fixed1(yaw_before) << "->" << fixed1(yaw_after)
                       << " delta=" << signedFixed1(yaw_delta)
                       << " err=" << signedFixed1(final_yaw_error) << " pos=" << fixed1(pos_before)
                       << "->" << fixed1(pos_after)
                       << " steer[tof=" << signedFixed1(tof_steer_degps)
                       << " src=" << tof_wall::sourceName(wall_state.source)
                       << " side=" << signedFixed1(wall_state.side_error_norm)
                       << " imu=" << signedFixed1(imu_steer_degps)
                       << " herr=" << signedFixed1(motion_->headingHoldErrorDeg())
                       << " total=" << signedFixed1(total_steer_degps)
                       << " peak=" << signedFixed1(steer_stats.peak_degps)
                       << " avg=" << signedFixed1(steer_stats.average_degps)
                       << " samples=" << steer_stats.samples
                       << " src_changes=" << steer_stats.source_changes
                       << " both=" << steer_stats.both_samples
                       << " single=" << steer_stats.single_samples << "]deg/s";
        }
        else
#endif
        {
            action_log << " yaw=NA delta=NA err=NA pos=SIM";
        }
        logSearchStepInfo(action_log.str());

        if (haltRequested())
            return false;
        ++step;
    }
}

void Mouse::log_action_status(const std::string& action, Cell* cell,
                              const std::string& position_text)
{
    if (cell == nullptr || maze_mouse_ == nullptr)
        return;

    logSearchStepInfo(
        "{" + action + " [" + std::to_string(cell->x()) + "," + std::to_string(cell->y()) + "] " +
        headingUpper(maze_mouse_->currentDirection()) + " " + (wallLeft() ? "L" : "-") +
        (wallFront() ? "F" : "-") + (wallRight() ? "R" : "-") + " @" + position_text + "}");
}

bool Mouse::search_maze()
{
    if (maze_mouse_ == nullptr)
        return false;

    LOG_INFO("search_maze: search to goal, return to start, face north");
    m_handStart = true;
    state_      = State::SEARCHING;
    set_heading("n");

    if (!search_to(goal_cells_))
        return false;

    AStar              a_star(maze_mouse_);
    std::vector<Cell*> return_path = a_star.cellPath({start_cell_}, /*diagonals=*/false,
                                                     /*pass_goals=*/true);
    if (return_path.empty())
    {
        LOG_ERROR("search_maze: no route to start");
        panic();
        return false;
    }

    const std::string return_heading =
        headingToNeighbor(maze_mouse_->currentCell(), return_path.front());
    if (!return_heading.empty() && !turn_to_face(return_heading))
        return false;

    m_handStart = false;
    if (!search_to({start_cell_}))
        return false;

    if (!turn_to_face("n"))
        return false;

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->stop();
        motion_->disable_drive();
    }
#endif
    state_ = State::FINISHED;
    return true;
}

bool Mouse::cheese_hunt(const std::array<int, 2>& start_cell, BeaconIr* beacon_ir)
{
    if (maze_mouse_ == nullptr)
        return false;

    auto beacon_found = [&]() -> bool
    {
#ifndef SIMULATOR_BUILD
        if (!run_on_simulator && beacon_ir != nullptr && beacon_ir->pollAndDeactivate())
        {
            stage_led::setBeaconFound();
            return true;
        }
#else
        (void)beacon_ir;
#endif
        return false;
    };

    auto return_to_start = [&]() -> bool
    {
        const std::vector<std::array<int, 2>> start_goal = {start_cell};
        if (atAnyGoal(maze_mouse_, start_goal))
            return turn_to_face("n");

        AStar              a_star(maze_mouse_);
        std::vector<Cell*> return_path =
            a_star.cellPath(start_goal, /*diagonals=*/false, /*pass_goals=*/true);
        if (return_path.empty())
        {
            LOG_ERROR("Cheese Hunt: no route back to start");
            return false;
        }

        const std::string return_heading =
            headingToNeighbor(maze_mouse_->currentCell(), return_path.front());
        if (!return_heading.empty() && !turn_to_face(return_heading))
            return false;

        m_handStart = false;
        if (!search_to(start_goal))
            return false;
        return turn_to_face("n");
    };

    MouseMotionSequenceGuard  motion_sequence(this);
    MouseMazeHeadingHoldGuard heading_hold(this);
    state_ = State::SEARCHING;
    maze_mouse_->setMazeMask(MASK_OPEN);

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator)
    {
        if (motion_ == nullptr)
            return false;
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        motion_->resetWallSteeringStats();
        begin_search_front_latch();
        float sensing_wait_position_mm = SENSING_POSITION_MM - HALF_CELL_MM;
        if (m_handStart)
        {
            sensing_wait_position_mm =
                START_CENTER_DISTANCE_MM + (SENSING_POSITION_MM - HALF_CELL_MM);
            LOG_INFO(
                "Cheese Hunt: hand_start chained_distance_mm=" + fixed1(sensing_wait_position_mm) +
                " sensing_position_mm=" + fixed1(SENSING_POSITION_MM));
            apply_maze_heading_hold();
            motion_->move(sensing_wait_position_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
            m_handStart = false;
        }
        else
        {
            LOG_INFO("Cheese Hunt: center_start distance_mm=" + fixed1(sensing_wait_position_mm));
            apply_maze_heading_hold();
            motion_->move(sensing_wait_position_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
        }
        if (!wait_until_position(sensing_wait_position_mm))
        {
            clear_search_front_latch();
            return false;
        }
        finish_search_front_latch();
        // Re-label into the cell frame but keep the distance already past the
        // wait point (see move_ahead()).
        motion_->set_position(SENSING_POSITION_MM +
                              (motion_->position() - sensing_wait_position_mm));
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif
    if (run_on_simulator)
        simulatorResponse("moveForward");

    const int max_steps = mazeWidth() * mazeHeight() * 8;
    for (int step = 1; step <= max_steps; ++step)
    {
        maze_mouse_->moveForward(1);
        Cell* current = maze_mouse_->currentCell();
        if (current == nullptr)
            return false;

        update_map();
        logSearchStepInfo("Cheese Hunt step=" + std::to_string(step) + " cell=(" +
                          std::to_string(current->x()) + "," + std::to_string(current->y()) +
                          ") heading=" + headingUpper(maze_mouse_->currentDirection()));

        if (beacon_found())
        {
            LOG_INFO("Cheese Hunt: beacon deactivated at (" + std::to_string(current->x()) + "," +
                     std::to_string(current->y()) + ")");
            if (!stopAtCentre())
                return false;
            if (!adjustPosition())
                return false;
            state_ = State::FINISHED;
            return return_to_start();
        }

        std::vector<Cell*> cell_path = pathToNearestUnexplored(maze_mouse_);
        if (cell_path.empty())
        {
            LOG_ERROR("Cheese Hunt: beacon not found");
            if (!stopAtCentre())
                return false;
            if (!adjustPosition())
                return false;
            state_ = State::FINISHED;
            return return_to_start();
        }

        std::string lfr =
            PathConverter::buildLFR(current, maze_mouse_->currentDirectionArray(), cell_path);
        std::string second;
        std::string action = firstToken(lfr, &second);
        logSearchStepInfo("Cheese Hunt action=" + action + " lfr=" + lfr);

        if (action == "F")
            move_ahead();
        else if ((action == "L" && second == "L") || (action == "R" && second == "R") ||
                 action == "B")
            turn_back();
        else if (action == "L")
            turn_left();
        else if (action == "R")
            turn_right();
        else
        {
            LOG_ERROR("Cheese Hunt: unsupported LFR action: " + action);
            return false;
        }

        if (haltRequested())
            return false;
    }

    LOG_ERROR("Cheese Hunt: step limit reached before beacon found");
    state_ = State::FINISHED;
    return return_to_start();
}

bool Mouse::turn_to_face(const std::string& heading)
{
    if (maze_mouse_ == nullptr)
        return false;

    const int current_index = maze_mouse_->findDirectionIndex(maze_mouse_->currentDirection());
    const int target_index  = maze_mouse_->findDirectionIndex(heading);
    if (current_index < 0 || target_index < 0)
        return false;

    int half_steps_right = (target_index - current_index + 8) % 8;
    if (half_steps_right == 0)
        return true;

    bool turn_ok = false;
    if (half_steps_right == 2)
        turn_ok = turn_IP90R();
    else if (half_steps_right == 6)
        turn_ok = turn_IP90L();
    else if (half_steps_right == 4)
        turn_ok = turn_IP180();
    else
        return false;

    if (!turn_ok)
        return false;
    maze_mouse_->setDirection(heading);
    return !haltRequested();
}

void Mouse::blink(int count)
{
#ifndef SIMULATOR_BUILD
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < count; ++i)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }
#else
    (void)count;
#endif
}

void Mouse::panic()
{
    LOG_ERROR("panic: unrecoverable mouse error; stopping drive");
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        motion_->emergency_stop();
        motion_->disable_drive();
    }
    for (int i = 0; i < 5 && !haltRequested(); ++i)
        blink(1);
#endif
    state_ = State::FINISHED;
}

bool Mouse::run(float distance_mm)
{
    if (maze_mouse_ == nullptr)
        return false;
    m_handStart = true;
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->reset_drive_system();
#endif
    if (!start_center())
        return false;
    return move_mm(distance_mm);
}

bool Mouse::run_to(const std::vector<std::array<int, 2>>& goals, bool diagonalized)
{
    if (maze_mouse_ == nullptr)
        return false;

    const MazeMask previous_mask = maze_mouse_->mazeMask();
    maze_mouse_->setMazeMask(MASK_CLOSED);

    AStar              a_star(maze_mouse_);
    std::vector<Cell*> cell_path = a_star.cellPath(goals, /*diagonals=*/false, /*pass_goals=*/true);
    if (cell_path.empty())
    {
        maze_mouse_->setMazeMask(previous_mask);
        return false;
    }

    Cell* current = maze_mouse_->currentCell();
    if (current == nullptr || !current->explored())
    {
        maze_mouse_->setMazeMask(previous_mask);
        return false;
    }
    for (Cell* cell : cell_path)
    {
        if (cell == nullptr || !cell->explored())
        {
            maze_mouse_->setMazeMask(previous_mask);
            return false;
        }
    }

    std::string lfr = PathConverter::buildLFR(maze_mouse_->currentCell(),
                                              maze_mouse_->currentDirectionArray(), cell_path);
    if (diagonalized)
        lfr = Diagonalizer::diagonalize(lfr);

    executeSequence(lfr);
    maze_mouse_->setMazeMask(previous_mask);
    return !haltRequested();
}

bool Mouse::getRandomBool()
{
    return std::rand() % 2 == 0;
}

std::string Mouse::randomHeading()
{
    const bool left_wall  = see_left_wall();
    const bool right_wall = see_right_wall();
    const bool front_wall = see_front_wall();

    if (left_wall && right_wall && front_wall)
        return "B";
    if (left_wall && right_wall)
        return "F";
    if (right_wall && front_wall)
        return "L";
    if (left_wall && front_wall)
        return "R";
    if (left_wall)
        return getRandomBool() ? "R" : "F";
    if (right_wall)
        return getRandomBool() ? "L" : "F";
    return getRandomBool() ? "L" : "R";
}

bool Mouse::follow_to(const std::vector<std::array<int, 2>>& goals)
{
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard  motion_sequence(this);
    MouseMazeHeadingHoldGuard heading_hold(this);
    state_ = State::SEARCHING;

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        if (m_handStart)
        {
            if (!start_center())
                return false;
            m_handStart = false;
        }
        motion_->set_position(HALF_CELL_MM);
        apply_maze_heading_hold();
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
        if (!waitForMotion())
            return false;
        motion_->set_position(SENSING_POSITION_MM);
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif
    if (run_on_simulator)
        simulatorResponse("moveForward");

    const int max_steps = mazeWidth() * mazeHeight() * 8;
    for (int step = 0; step < max_steps && !atAnyGoal(maze_mouse_, goals); ++step)
    {
        maze_mouse_->moveForward(1);
        update_map();
        logSearchStepInfo("follow_to step=" + std::to_string(step + 1) + " action=left-wall");
        if (atAnyGoal(maze_mouse_, goals))
        {
            // We are entering the target cell; let the final stop centre it.
        }
        else if (!see_left_wall())
            turn_left();
        else if (!see_front_wall())
            move_ahead();
        else if (!see_right_wall())
            turn_right();
        else
            turn_back();

        if (haltRequested())
            return false;
    }

    if (!atAnyGoal(maze_mouse_, goals))
    {
        LOG_ERROR("follow_to: target not reached before step limit");
        return false;
    }
    if (!stopAtCentre())
        return false;
    if (!adjustPosition())
        return false;
    state_ = State::FINISHED;
    return true;
}

bool Mouse::wander_to(const std::vector<std::array<int, 2>>& goals)
{
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard  motion_sequence(this);
    MouseMazeHeadingHoldGuard heading_hold(this);
    state_ = State::SEARCHING;

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        if (m_handStart)
        {
            if (!start_center())
                return false;
            m_handStart = false;
        }
        motion_->set_position(HALF_CELL_MM);
        apply_maze_heading_hold();
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
        if (!waitForMotion())
            return false;
        motion_->set_position(SENSING_POSITION_MM);
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif
    if (run_on_simulator)
        simulatorResponse("moveForward");

    const int max_steps = mazeWidth() * mazeHeight() * 8;
    for (int step = 0; step < max_steps && !atAnyGoal(maze_mouse_, goals); ++step)
    {
        maze_mouse_->moveForward(1);
        update_map();
        const std::string action = randomHeading();
        logSearchStepInfo("wander_to step=" + std::to_string(step + 1) + " action=" + action);
        if (atAnyGoal(maze_mouse_, goals))
        {
            // We are entering the target cell; let the final stop centre it.
        }
        else if (action == "F")
            move_ahead();
        else if (action == "L")
            turn_left();
        else if (action == "R")
            turn_right();
        else
            turn_back();

        if (haltRequested())
            return false;
    }

    if (!atAnyGoal(maze_mouse_, goals))
    {
        LOG_ERROR("wander_to: target not reached before step limit");
        return false;
    }
    if (!stopAtCentre())
        return false;
    if (!adjustPosition())
        return false;
    state_ = State::FINISHED;
    return true;
}

bool Mouse::test_SS90E(bool turn_left)
{
    // mazerunner-core test_SS90E(), driven through smooth_turn_step(): the very
    // step the search loop takes. Arrival speed, steering and heading-hold
    // handoff, the arc and the exit to SENSING_POSITION_MM are the search code
    // itself, so a turn tuned here is tuned for search.
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard motion_sequence(this);
#ifndef SIMULATOR_BUILD
    float start_yaw_deg = 0.0f;
    if (!run_on_simulator && motion_ != nullptr)
    {
        // Same as search_to(): never steer from zero speed, run to the first
        // sensing point at search speed while holding the start heading.
        motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
        if (m_handStart)
        {
            if (!start_center())
                return false;
            m_handStart = false;
        }
        // Search holds the maze heading on straights; on a square start that
        // is the yaw the robot was placed at.
        start_yaw_deg = motion_->yaw_deg();
        motion_->set_heading_hold(start_yaw_deg);
        motion_->set_position(HALF_CELL_MM);
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_SEARCH_ACCEL_MMPS2);
        if (!waitForMotion())
            return false;
        motion_->set_position(SENSING_POSITION_MM);
        motion_->set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    }
#endif

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->start_trace();
#endif
    smooth_turn_step(turn_left);
    maze_mouse_->turn45Steps(turn_left ? -2 : 2);

#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr && !haltRequested())
    {
        // Read the turn's yaw error now: the run-out below holds the new
        // heading and would hide it.
        const float target_yaw_deg =
            normalizeYawDelta(start_yaw_deg + (turn_left ? 90.0f : -90.0f));
        const float turn_error_deg = normalizeYawDelta(target_yaw_deg - motion_->yaw_deg());
        LOG_INFO("SS90E " + std::string(turn_left ? "L" : "R") +
                 " start_yaw=" + fixed1(start_yaw_deg) + " target_yaw=" + fixed1(target_yaw_deg) +
                 " yaw=" + fixed1(motion_->yaw_deg()) + " turn_yaw_error=" +
                 signedFixed1(turn_error_deg) + " position_mm=" + fixed1(motion_->position()));

        // mazerunner-core: run on two cells to a stop. The robot rests at the
        // sensing point two cells on, so its offset there is the turn's.
        motion_->set_heading_hold(target_yaw_deg);
        motion_->move(2.0f * CELL_SIZE_MM, cruise_speed_mmps_, 0.0f, ROBOT_SEARCH_ACCEL_MMPS2);
        const bool run_out_ok = waitForMotion();
        motion_->stop_trace();
        print_turn_trace(target_yaw_deg);
        if (!run_out_ok)
            return false;
    }
#endif
    return !haltRequested();
}

void Mouse::print_turn_trace(float target_yaw_deg)
{
#ifndef SIMULATOR_BUILD
    // One CSV row per trace sample, paced to the Bluetooth UART: the 4 KB TX
    // ring holds ~60 rows, so an unpaced dump drops the rest. haltRequested()
    // drains the console; a ~75 byte row takes ~6.5 ms at 115200 baud.
    const uint16_t count = motion_->traceCount();
    LOG_INFO("TRACE n="
             << count << " dt_ms=" << motion_->traceDtMs()
             << " cols=t_ms,target_deg,yaw_deg,rot_err_deg,rot_v,hold_err_deg,"
                "wall_degps,hold_degps,fwd_mmps,left_mmps,right_mmps,left_v,right_v,enc,tof_l_mm,"
                "tof_f_mm,tof_r_mm,pos_mm");
    Log::drainBluetooth();
    for (uint16_t i = 0; i < count; ++i)
    {
        const Motion::TraceSample& s = motion_->traceSample(i);
        char                       row[160];
        std::snprintf(row, sizeof(row),
                      "T,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.0f,%.0f,%.2f,%.2f,%d,%.0f,%."
                      "0f,%.0f,%.1f",
                      i * motion_->traceDtMs(), static_cast<double>(s.target_deg),
                      static_cast<double>(s.yaw_deg), static_cast<double>(s.rot_error_deg),
                      static_cast<double>(s.rot_volts), static_cast<double>(s.hold_err_deg),
                      static_cast<double>(s.wall_degps), static_cast<double>(s.hold_degps),
                      static_cast<double>(s.fwd_mmps), static_cast<double>(s.left_mmps),
                      static_cast<double>(s.right_mmps), static_cast<double>(s.left_volts),
                      static_cast<double>(s.right_volts), s.encoder_rotation ? 1 : 0,
                      static_cast<double>(s.left_tof_mm), static_cast<double>(s.front_tof_mm),
                      static_cast<double>(s.right_tof_mm), static_cast<double>(s.pos_mm));
        LOG_INFO(row);
        for (int k = 0; k < 4; ++k)
        {
            if (haltRequested())
                return;
            sleep_ms(2);
        }
    }
    // The SS90E line reads yaw while the IMU still lags the arc by ~40 ms.
    // Score the turn 100 ms after the rotation profile reaches its target,
    // once that lag has drained. A NaN target (CLI TRACE) has no turn.
    const uint16_t kSettleSamples   = 100 / motion_->traceDtMs();
    float          final_target_deg = 0.0f;
    for (uint16_t i = 0; i < count; ++i)
        final_target_deg =
            std::max(final_target_deg, std::fabs(motion_->traceSample(i).target_deg));
    for (uint16_t i = 0; i < count && !std::isnan(target_yaw_deg); ++i)
    {
        if (std::fabs(motion_->traceSample(i).target_deg) < final_target_deg - 0.2f)
            continue;
        const uint16_t j = i + kSettleSamples;
        if (j < count)
        {
            LOG_INFO(
                "TRACE turn_yaw_error_settled="
                << signedFixed1(normalizeYawDelta(target_yaw_deg - motion_->traceSample(j).yaw_deg))
                << " at_ms=" << j * motion_->traceDtMs());
        }
        break;
    }
    LOG_INFO("TRACE end");
    Log::drainBluetooth();
#else
    (void)target_yaw_deg;
#endif
}

void Mouse::show_sensor_calibration()
{
    serviceSensors();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        const float               left_mm    = motion_->leftDistance();
        const float               front_mm   = motion_->frontDistance();
        const float               right_mm   = motion_->rightDistance();
        const float               left_norm  = left_mm * TOF_LEFT_SCALE;
        const float               right_norm = right_mm * TOF_RIGHT_SCALE;
        const tof_wall::WallState state =
            tof_wall::evaluate(left_mm, front_mm, right_mm, motion_->steeringMode());
        const float preview_degps =
            state.steering_allowed ? tof_wall::steeringAdjustmentDegps(state.side_error_norm, 0.0f)
                                   : 0.0f;

        LOG_INFO("Sensor static calibration");
        LOG_INFO("L raw=" + fixed1(left_mm) + " mm norm=" + fixed1(left_norm) +
                 " wall=" + std::to_string(state.left_wall ? 1 : 0));
        LOG_INFO("F raw=" + fixed1(front_mm) +
                 " mm wall=" + std::to_string(state.front_wall ? 1 : 0));
        LOG_INFO("R raw=" + fixed1(right_mm) + " mm norm=" + fixed1(right_norm) +
                 " wall=" + std::to_string(state.right_wall ? 1 : 0));
        LOG_INFO("Side src=" + std::string(tof_wall::sourceName(state.source)) +
                 " err=" + fixed1(state.side_error_norm) + " preview=" + fixed1(preview_degps) +
                 " deg/s mode=" + tof_wall::modeName(motion_->steeringMode()));
        return;
    }
#endif
    LOG_INFO(std::string("Sensor static calibration: walls=") + (wallLeft() ? "L" : "-") +
             (wallFront() ? "F" : "-") + (wallRight() ? "R" : "-"));
}

void Mouse::print_wall_sensors()
{
    serviceSensors();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        const tof_wall::WallState state =
            tof_wall::evaluate(motion_->leftDistance(), motion_->frontDistance(),
                               motion_->rightDistance(), motion_->steeringMode());
        LOG_INFO("wall_sensors: L=" + fixed1(motion_->leftDistance()) + " F=" +
                 fixed1(motion_->frontDistance()) + " R=" + fixed1(motion_->rightDistance()) +
                 " walls=" + (state.left_wall ? "L" : "-") + (state.front_wall ? "F" : "-") +
                 (state.right_wall ? "R" : "-") +
                 " mode=" + tof_wall::modeName(motion_->steeringMode()) + " src=" +
                 tof_wall::sourceName(state.source) + " err=" + fixed1(state.side_error_norm));
        return;
    }
#endif
    LOG_INFO(std::string("wall_sensors: walls=") + (wallLeft() ? "L" : "-") +
             (wallFront() ? "F" : "-") + (wallRight() ? "R" : "-"));
}

void Mouse::report_profile()
{
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        LOG_INFO("profile: pos=" + fixed1(motion_->position()) +
                 " vel=" + fixed1(motion_->velocity()) + " acc=" + fixed1(motion_->acceleration()) +
                 " angle=" + fixed1(motion_->angle()) + " omega=" + fixed1(motion_->omega()) +
                 " alpha=" + fixed1(motion_->alpha()) + " yaw=" + fixed1(motion_->yaw_deg()) +
                 " yaw_rate=" + fixed1(motion_->yaw_rate_degps()));
        return;
    }
#endif
    LOG_INFO("profile: simulator/no motion");
}

void Mouse::front_sensor_track_header()
{
    LOG_INFO("front_sensor_track: position_mm,front_mm,front_wall");
}

void Mouse::front_sensor_track()
{
    serviceSensors();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        LOG_INFO("front_sensor_track: " + fixed1(motion_->position()) + "," +
                 fixed1(motion_->frontDistance()) + "," + (wallFront() ? "1" : "0"));
        return;
    }
#endif
    LOG_INFO(std::string("front_sensor_track: SIM,") + (wallFront() ? "wall" : "open"));
}

void Mouse::report_sensor_track_header()
{
    LOG_INFO("sensor_track: yaw_deg,left_mm,front_mm,right_mm,left_wall,front_wall,right_wall,"
             "src,side_error_norm,steering_degps");
}

void Mouse::report_radial_track(bool use_raw)
{
    (void)use_raw;
    serviceSensors();
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        const tof_wall::WallState state =
            tof_wall::evaluate(motion_->leftDistance(), motion_->frontDistance(),
                               motion_->rightDistance(), motion_->steeringMode());
        LOG_INFO("sensor_track: " + fixed1(motion_->yaw_deg()) + "," +
                 fixed1(motion_->leftDistance()) + "," + fixed1(motion_->frontDistance()) + "," +
                 fixed1(motion_->rightDistance()) + "," + (state.left_wall ? "1" : "0") + "," +
                 (state.front_wall ? "1" : "0") + "," + (state.right_wall ? "1" : "0") + "," +
                 tof_wall::sourceName(state.source) + "," + fixed1(state.side_error_norm) + "," +
                 fixed1(motion_->wallSteeringAdjustmentDegps()));
        return;
    }
#endif
    LOG_INFO("sensor_track: simulator/no motion");
}

void Mouse::conf_log_front_sensor()
{
#ifndef SIMULATOR_BUILD
    if (run_on_simulator || motion_ == nullptr)
        return;
    MouseMotionSequenceGuard motion_sequence(this);
    motion_->reset_drive_system();
    motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
    front_sensor_track_header();
    motion_->start_move(-200.0f, 100.0f, 0.0f, 500.0f);
    while (!motion_->move_finished() && !haltRequested())
    {
        front_sensor_track();
        sleep_ms(TOF_MEASUREMENT_PERIOD_MS);
    }
    front_sensor_track();
    motion_->reset_drive_system();
    motion_->disable_drive();
#else
    front_sensor_track_header();
    front_sensor_track();
#endif
}

void Mouse::conf_sensor_spin_calibrate()
{
#ifndef SIMULATOR_BUILD
    if (run_on_simulator || motion_ == nullptr)
        return;
    MouseMotionSequenceGuard motion_sequence(this);
    motion_->reset_drive_system();
    motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
    report_sensor_track_header();
    motion_->start_turn(360.0f, 180.0f, 0.0f, 1800.0f);
    while (!motion_->turn_finished() && !haltRequested())
    {
        report_radial_track(false);
        sleep_ms(TOF_MEASUREMENT_PERIOD_MS);
    }
    report_radial_track(false);
    motion_->reset_drive_system();
    motion_->disable_drive();
#else
    report_sensor_track_header();
    report_radial_track(false);
#endif
}

void Mouse::conf_edge_detection()
{
#ifndef SIMULATOR_BUILD
    if (run_on_simulator || motion_ == nullptr)
        return;
    MouseMotionSequenceGuard motion_sequence(this);
    motion_->reset_drive_system();
    motion_->set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
    serviceSensors();
    bool left_was_wall  = wallLeft();
    bool right_was_wall = wallRight();
    bool left_logged    = false;
    bool right_logged   = false;
    LOG_INFO("edge_detection: position_mm,left_wall,right_wall");
    motion_->start_move(150.0f, 100.0f, 0.0f, 500.0f);
    while (!motion_->move_finished() && !haltRequested())
    {
        serviceSensors();
        const bool left_now  = wallLeft();
        const bool right_now = wallRight();
        if (!left_logged && left_was_wall && !left_now)
        {
            LOG_INFO("edge_detection: left_edge_mm=" + fixed1(motion_->position()));
            left_logged = true;
        }
        if (!right_logged && right_was_wall && !right_now)
        {
            LOG_INFO("edge_detection: right_edge_mm=" + fixed1(motion_->position()));
            right_logged = true;
        }
        left_was_wall  = left_now;
        right_was_wall = right_now;
        sleep_ms(TOF_MEASUREMENT_PERIOD_MS);
    }
    LOG_INFO("edge_detection: done left_logged=" + std::to_string(left_logged ? 1 : 0) +
             " right_logged=" + std::to_string(right_logged ? 1 : 0));
    motion_->reset_drive_system();
    motion_->disable_drive();
#else
    LOG_INFO("edge_detection: simulator/no motion");
#endif
}

void Mouse::arcTurnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft90");
#ifndef SIMULATOR_BUILD
    else
    {
        smooth_turn_step(/*turn_left=*/true);
    }
#endif
    maze_mouse_->turn45Steps(-2);
}

void Mouse::arcTurnRight90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight90");
#ifndef SIMULATOR_BUILD
    else
    {
        smooth_turn_step(/*turn_left=*/false);
    }
#endif
    maze_mouse_->turn45Steps(2);
}

void Mouse::arcTurnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft45");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        // 45-deg arc with a 45 mm radius — same shape startArcTurn produced.
        const float radius_mm = 45.0f;
        const float arc_mm    = 45.0f * (M_PI / 180.0f) * radius_mm;
        suspend_maze_heading_hold();
        motion_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                            ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        motion_->start_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                            ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        if (!waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(-1);
}

void Mouse::arcTurnRight45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight45");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        const float radius_mm = 45.0f;
        const float arc_mm    = 45.0f * (M_PI / 180.0f) * radius_mm;
        suspend_maze_heading_hold();
        motion_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                            ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        motion_->start_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                            ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        if (!waitForMotion())
            return;
    }
#endif
    maze_mouse_->turn45Steps(1);
}

void Mouse::executeSequence(const std::string& sequence)
{
    MouseMazeHeadingHoldGuard heading_hold(this);
    std::istringstream        ss(sequence);
    std::string               token;
    std::vector<std::string>  tokens;

    while (std::getline(ss, token, '#'))
    {
        if (token.empty())
            continue;

        tokens.push_back(uppercaseToken(token));
    }

    for (size_t i = 0; i < tokens.size(); ++i)
    {
        token = tokens[i];

        if (token == "F")
        {
            if (movement_style_ == MovementStyle::Smooth)
            {
                move_ahead();
                maze_mouse_->moveForward(1);
            }
            else
            {
                moveForward();
            }
            continue;
        }

        if (token == "FH")
        {
            moveForwardHalf();
            continue;
        }

        if (token == "L")
        {
            if (movement_style_ == MovementStyle::Smooth)
            {
                if (i + 1 < tokens.size() && tokens[i + 1] == "L")
                {
                    turn_back();
                    ++i;
                }
                else
                {
                    turn_left();
                }
            }
            else
            {
                turnLeft90();
            }
            continue;
        }

        if (token == "R")
        {
            if (movement_style_ == MovementStyle::Smooth)
            {
                if (i + 1 < tokens.size() && tokens[i + 1] == "R")
                {
                    turn_back();
                    ++i;
                }
                else
                {
                    turn_right();
                }
            }
            else
            {
                turnRight90();
            }
            continue;
        }

        if (token == "L45")
        {
            if (movement_style_ == MovementStyle::Smooth)
                arcTurnLeft45();
            else
                turnLeft45();
            continue;
        }

        if (token == "R45")
        {
            if (movement_style_ == MovementStyle::Smooth)
                arcTurnRight45();
            else
                turnRight45();
            continue;
        }

        if (token == "GMF" || token == "GFM")
        {
            if (token == "GFM")
                LOG_ERROR("Mouse: Treating diagonalizer token GFM as GMF");
            ghostMoveForward(1);
            continue;
        }

        if (token.size() > 1 && token[0] == 'F')
        {
            int steps = 0;
            if (parsePositiveInt(token.substr(1), steps))
            {
                if (movement_style_ == MovementStyle::Smooth)
                {
                    for (int step = 0; step < steps; ++step)
                    {
                        move_ahead();
                        maze_mouse_->moveForward(1);
                    }
                }
                else
                {
                    moveForward(steps);
                }
                continue;
            }
        }

        LOG_ERROR("Mouse: Unknown path token: " + token);
        return;
    }
}

void Mouse::setWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "setWall " << x << " " << y << " " << dir << '\n';

    Cell* cell = maze_mouse_->cellAt(x, y);
    if (cell)
    {
        maze_mouse_->setWallNESW(cell, dir[0]);
        if (dir.size() > 1)
            maze_mouse_->setWallNESW(cell, dir[1]);
    }
}

void Mouse::clearWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "clearWall " << x << " " << y << " " << dir << '\n';
}

void Mouse::setColor(int x, int y, char color)
{
    if (run_on_simulator)
        std::cout << "setColor " << x << " " << y << " " << color << '\n';
}

void Mouse::setPhaseColor(char color)
{
    phase_color_ = color;
}

char Mouse::phaseColor() const
{
    return phase_color_;
}

void Mouse::clearColor(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearColor " << x << " " << y << '\n';
}

void Mouse::clearAllColor()
{
    if (run_on_simulator)
        std::cout << "clearAllColor" << '\n';
}

void Mouse::setText(int x, int y, const std::string& text)
{
    if (run_on_simulator)
        std::cout << "setText " << x << " " << y << " " << text << '\n';
}

void Mouse::clearText(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearText " << x << " " << y << '\n';
}

void Mouse::clearAllText()
{
    if (run_on_simulator)
        std::cout << "clearAllText" << '\n';
}

std::string Mouse::simulatorResponse(const std::string& cmd)
{
    std::cout << cmd << '\n';
    std::string resp;
    std::getline(std::cin, resp);
    return resp;
}

bool Mouse::simulatorBool(const std::string& cmd)
{
    return simulatorResponse(cmd) == "true";
}

void Mouse::setUp(std::array<int, 2> start, std::vector<std::array<int, 2>> goals)
{
    start_cell_ = start;
    goal_cells_ = goals;
    if (maze_mouse_ != nullptr)
        maze_mouse_->reset(start_cell_, "n", goal_cells_);

    clearAllColor();
    clearAllText();

    // Add boundary walls
    for (int i = 0; i < mazeWidth(); i++)
    {
        setWall(i, 0, "s");
        setWall(i, mazeHeight() - 1, "n");
    }
    for (int j = 0; j < mazeHeight(); j++)
    {
        setWall(0, j, "w");
        setWall(mazeWidth() - 1, j, "e");
    }

    // UKMARS start-cell convention: the mouse starts in the south-west cell
    // facing north, with the east wall present and the north side known open.
    setWall(start[0], start[1], "e");
    if (Cell* start_cell = maze_mouse_->cellAt(start[0], start[1]))
    {
        start_cell->updateWallState('N', EXIT);
        start_cell->markExplored();
    }

    // Add grid labels
    for (int i = 0; i < mazeWidth(); i++)
    {
        for (int j = 0; j < mazeHeight(); j++)
        {
            setText(i, j, std::to_string(i) + "," + std::to_string(j));
        }
    }

    // Mark start
    setColor(start[0], start[1], 'B');
    setText(start[0], start[1], "Start");

    // Mark goals
    for (auto& goal : goals)
    {
        setColor(goal[0], goal[1], 'G');
        setText(goal[0], goal[1], "Goal");
    }
}

void Mouse::printMaze()
{
    std::cout << mazeString();
}

std::string Mouse::mazeString()
{
    std::stringstream ss;
    ss << "Maze:\n";

    for (int col = 0; col < mazeWidth(); col++)
    {
        ss << "+---";
    }
    ss << "+\n";

    for (int row = mazeHeight() - 1; row >= 0; row--)
    {
        ss << printMazeRow(row);
    }
    return ss.str();
}

std::string Mouse::printMazeRow(int row)
{
    std::stringstream vertical, horizontal;

    vertical << "|";
    for (int col = 0; col < mazeWidth(); col++)
    {
        Cell* cell = maze_mouse_->cellAt(col, row);
        if (cell == nullptr)
        {
            vertical << "   ?";
            horizontal << "+???";
            continue;
        }

        vertical << (cell->hasWall('E') ? "   |" : "    ");
        horizontal << (cell->hasWall('S') ? "+---" : "+   ");
    }

    horizontal << "+\n";

    std::stringstream result;
    result << vertical.str() << "\n" << horizontal.str();
    return result.str();
}
