#include "app/mouse.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef SIMULATOR_BUILD
#include "pico/stdlib.h"

#include "config/motion.h"
#include "control/motion.h"
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

std::string headingUpper(const std::string& heading)
{
    std::string out = heading;
    for (char& c : out)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
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
} // namespace

Mouse::Mouse(MazeMouse* maze_mouse) : run_on_simulator(false), maze_mouse_(maze_mouse)
{
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

void Mouse::waitForMotion()
{
#ifndef SIMULATOR_BUILD
    // UKMARS busy-wait: spin while the controller (running in the 500 Hz
    // hardware timer ISR) advances the trapezoidal profile to completion.
    // 2 ms cadence matches mazerunner-core's `delay(2)` inside
    // Profile::wait_until_finished.
    if (motion_ == nullptr)
        return;
    while (!motion_->move_finished() || !motion_->turn_finished())
    {
        serviceSensors();
        if (haltRequested())
        {
            motion_->emergency_stop();
            return;
        }
        sleep_ms(2);
    }
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
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->begin_motion_sequence();
#endif
}

void Mouse::end_motion_sequence()
{
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
        motion_->end_motion_sequence();
#endif
}

void Mouse::moveForwardHalf()
{
    if (run_on_simulator)
        simulatorResponse("moveForwardHalf");
#ifndef SIMULATOR_BUILD
    else if (motion_ != nullptr)
    {
        motion_->move(HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
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
    LOG_INFO("MOTION move_mm: distance_mm=" + fixed1(distance_mm) +
             " speed_mmps=" + fixed1(ROBOT_MAX_SEARCH_SPEED_MMPS) +
             " accel_mmps2=" + fixed1(ROBOT_BASE_ACCEL_MMPS2));
    motion_->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    waitForMotion();
    return !haltRequested();
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
    motion_->move(distance_mm, speed_mmps, 0.0f, accel_mmps2);
    waitForMotion();
    return !haltRequested();
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
        motion_->move(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
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
        motion_->move(steps * CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f,
                      ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
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
        motion_->spin_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
        motion_->spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
        motion_->spin_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
        motion_->spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
        motion_->spin_turn(static_cast<float>(degrees), ROBOT_MAX_TURN_SPEED_DEGPS,
                           ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
    motion_->adjust_forward_position(-CELL_SIZE_MM);
    if (!wait_until_position(SENSING_POSITION_MM))
        return;
    motion_->set_position(SENSING_POSITION_MM);
#endif
}

void Mouse::turn_left()
{
    if (movement_style_ == MovementStyle::Smooth)
    {
        turn_smooth(SS90EL);
#ifndef SIMULATOR_BUILD
        if (!run_on_simulator && motion_ != nullptr)
            motion_->set_position(SENSING_POSITION_MM);
#endif
    }
    else
    {
        if (!stopAtCentre())
            return;
        if (!adjustPosition())
            return;
        turn_IP90L();
        if (run_on_simulator)
            simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
        if (!run_on_simulator && motion_ != nullptr)
        {
            motion_->set_position(HALF_CELL_MM);
            motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
            waitForMotion();
            motion_->set_position(SENSING_POSITION_MM);
        }
#endif
    }
    maze_mouse_->turn45Steps(-2);
}

void Mouse::turn_right()
{
    if (movement_style_ == MovementStyle::Smooth)
    {
        turn_smooth(SS90ER);
#ifndef SIMULATOR_BUILD
        if (!run_on_simulator && motion_ != nullptr)
            motion_->set_position(SENSING_POSITION_MM);
#endif
    }
    else
    {
        if (!stopAtCentre())
            return;
        if (!adjustPosition())
            return;
        turn_IP90R();
        if (run_on_simulator)
            simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
        if (!run_on_simulator && motion_ != nullptr)
        {
            motion_->set_position(HALF_CELL_MM);
            motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
            waitForMotion();
            motion_->set_position(SENSING_POSITION_MM);
        }
#endif
    }
    maze_mouse_->turn45Steps(2);
}

void Mouse::turn_back()
{
    if (!stopAtCentre())
        return;
    if (!adjustPosition())
        return;
    turn_IP180();
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    if (!run_on_simulator && motion_ != nullptr)
    {
        motion_->set_position(HALF_CELL_MM);
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
        motion_->set_position(SENSING_POSITION_MM);
    }
#endif
    maze_mouse_->turn45Steps(4);
}

void Mouse::turn_smooth(int turn_id)
{
    if (run_on_simulator)
    {
        simulatorResponse((turn_id & 1) ? "arcTurnRight90" : "arcTurnLeft90");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return;
    // Motion::turn_smooth starts simultaneous fwd + rot profiles using the
    // SMOOTH_TURN_PARAMS table — same emergent-arc shape as mazerunner.
    motion_->turn_smooth(turn_id);
    waitForMotion();
#endif
}

void Mouse::turn_IP180()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        simulatorResponse("turnRight");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return;
    motion_->turn_IP180();
    waitForMotion();
#endif
}

void Mouse::turn_IP90R()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return;
    motion_->turn_IP90R();
    waitForMotion();
#endif
}

void Mouse::turn_IP90L()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnLeft");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return;
    motion_->turn_IP90L();
    waitForMotion();
#endif
}

bool Mouse::wait_until_position(float position_mm)
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    while (motion_->position() < position_mm)
    {
        serviceSensors();
        if (haltRequested())
        {
            motion_->emergency_stop();
            return false;
        }
        sleep_ms(2);
    }
    return motion_->position() >= position_mm - 0.125f && !haltRequested();
#else
    (void)position_mm;
    return true;
#endif
}

bool Mouse::stopAtCentre()
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (motion_ == nullptr)
        return false;
    const float centre_position_mm = CELL_SIZE_MM + HALF_CELL_MM;
    const float distance_mm        = centre_position_mm - motion_->position();
    LOG_INFO("stopAtCentre: position_mm=" + fixed1(motion_->position()) +
             " distance_mm=" + fixed1(distance_mm));
    motion_->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    waitForMotion();
    if (haltRequested())
        return false;
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

    serviceSensors();
    const float front_mm = motion_->frontDistance();
    if (front_mm <= 0.0f || front_mm >= TOF_OUT_OF_RANGE_MM || !wallFront())
        return true;

    float correction_mm = front_mm - FRONT_REFERENCE_MM;
    if (std::fabs(correction_mm) <= FRONT_CORRECTION_TOLERANCE_MM)
        return true;

    if (correction_mm > FRONT_CORRECTION_STEP_MM)
        correction_mm = FRONT_CORRECTION_STEP_MM;
    if (correction_mm < -FRONT_CORRECTION_STEP_MM)
        correction_mm = -FRONT_CORRECTION_STEP_MM;

    LOG_INFO("adjustPosition: front_mm=" + fixed1(front_mm) +
             " correction_mm=" + fixed1(correction_mm));
    motion_->move(correction_mm, FRONT_CORRECTION_SPEED_MMPS, 0.0f, FRONT_CORRECTION_ACCEL_MMPS2);
    waitForMotion();
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

    cell->updateWallState(front[0], wallFront() ? WALL : EXIT);
    cell->updateWallState(left[0], wallLeft() ? WALL : EXIT);
    cell->updateWallState(right[0], wallRight() ? WALL : EXIT);
    LOG_INFO("update_map: cell=(" + std::to_string(x) + "," + std::to_string(y) + ") heading=" +
             headingUpper(maze_mouse_->currentDirection()) + " walls=" + (wallLeft() ? "L" : "-") +
             (wallFront() ? "F" : "-") + (wallRight() ? "R" : "-"));

    if (run_on_simulator)
    {
        if (wallFront())
            setWall(x, y, front);
        if (wallLeft())
            setWall(x, y, left);
        if (wallRight())
            setWall(x, y, right);
    }

    cell->markExplored();
}

bool Mouse::search_to(const std::vector<std::array<int, 2>>& goals)
{
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard motion_sequence(this);
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
        if (m_handStart)
        {
            LOG_INFO("search_to: hand_start distance_mm=" + fixed1(START_CENTER_DISTANCE_MM) +
                     " set_position_mm=" + fixed1(HALF_CELL_MM) +
                     " sensing_position_mm=" + fixed1(SENSING_POSITION_MM));
            motion_->move(START_CENTER_DISTANCE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                          ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
            waitForMotion();
            if (haltRequested())
                return false;
            m_handStart = false;
        }
        motion_->set_position(HALF_CELL_MM);
        if (!wait_until_position(SENSING_POSITION_MM))
            return false;
        motion_->set_position(SENSING_POSITION_MM);
    }
#endif
    if (run_on_simulator)
        simulatorResponse("moveForward");

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
            LOG_INFO("search_to: goal reached at (" + std::to_string(current->x()) + "," +
                     std::to_string(current->y()) + ")");
            if (!stopAtCentre())
                return false;
            if (!adjustPosition())
                return false;
            state_ = State::FINISHED;
            return true;
        }

        AStar              a_star(maze_mouse_);
        std::vector<Cell*> cell_path = a_star.cellPath(goals, /*diagonals=*/false,
                                                       /*pass_goals=*/true);
        if (cell_path.empty())
        {
            LOG_ERROR("search_to: no path found from (" + std::to_string(current->x()) + "," +
                      std::to_string(current->y()) + ")");
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
        LOG_INFO("search_to step=" + std::to_string(step) + " cell=(" +
                 std::to_string(current->x()) + "," + std::to_string(current->y()) +
                 ") heading=" + headingUpper(maze_mouse_->currentDirection()) +
                 " entered=1 lfr=" + lfr + " action=" + action + " position_mm=" + position_text);
        log_action_status(action, current, position_text);

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

    LOG_INFO("{" + action + " [" + std::to_string(cell->x()) + "," + std::to_string(cell->y()) +
             "] " + headingUpper(maze_mouse_->currentDirection()) + " " + (wallLeft() ? "L" : "-") +
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

    if (half_steps_right == 2)
        turn_IP90R();
    else if (half_steps_right == 6)
        turn_IP90L();
    else if (half_steps_right == 4)
        turn_IP180();
    else
        return false;

    maze_mouse_->setDirection(heading);
    return !haltRequested();
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
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 5 && !haltRequested(); ++i)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }
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

    MouseMotionSequenceGuard motion_sequence(this);
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
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
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
        LOG_INFO("follow_to step=" + std::to_string(step + 1) + " action=left-wall");
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

    MouseMotionSequenceGuard motion_sequence(this);
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
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
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
        LOG_INFO("wander_to step=" + std::to_string(step + 1) + " action=" + action);
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

bool Mouse::test_SS90E()
{
    if (maze_mouse_ == nullptr)
        return false;

    MouseMotionSequenceGuard motion_sequence(this);
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
        motion_->move(SENSING_POSITION_MM - HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
        motion_->set_position(SENSING_POSITION_MM);
    }
#endif
    turn_smooth(SS90ER);
    maze_mouse_->turn45Steps(2);
    return !haltRequested();
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
                 " alpha=" + fixed1(motion_->alpha()));
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
        LOG_INFO("sensor_track: " + fixed1(motion_->angle()) + "," +
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
        turn_smooth(SS90EL);
#endif
    maze_mouse_->turn45Steps(-2);
}

void Mouse::arcTurnRight90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight90");
#ifndef SIMULATOR_BUILD
    else
        turn_smooth(SS90ER);
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
        motion_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                            ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        motion_->start_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                            ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
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
        motion_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                            ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        motion_->start_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                            ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    maze_mouse_->turn45Steps(1);
}

void Mouse::executeSequence(const std::string& sequence)
{
    std::istringstream       ss(sequence);
    std::string              token;
    std::vector<std::string> tokens;

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
