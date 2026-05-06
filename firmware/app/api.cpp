#include "app/api.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef SIMULATOR_BUILD
#include "pico/stdlib.h"

#include "config/motion.h"
#include "control/robot.h"
#endif
#include "common/log.h"
#include "config/geometry.h"
#include "config/smooth_turn.h"
#include "maze/maze.h"
#include "maze/mouse.h"

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

std::string fixed1(float value)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
    return buffer;
}
} // namespace

API::API(Mouse* mouse) : mouse_(mouse), run_on_simulator(false)
{
}

void API::waitForMotion()
{
#ifndef SIMULATOR_BUILD
    // UKMARS busy-wait: spin while the controller (running in the 500 Hz
    // hardware timer ISR) advances the trapezoidal profile to completion.
    // 2 ms cadence matches mazerunner-core's `delay(2)` inside
    // Profile::wait_until_finished.
    if (robot_ == nullptr)
        return;
    while (!robot_->move_finished() || !robot_->turn_finished())
    {
        if (haltRequested())
        {
            robot_->emergency_stop();
            return;
        }
        sleep_ms(2);
    }
#endif
}

int API::mazeWidth()
{
    return mouse_->mazeWidth();
}

int API::mazeHeight()
{
    return mouse_->mazeHeight();
}

bool API::wallLeft()
{
    return run_on_simulator ? simulatorBool("wallLeft") : false;
}

bool API::wallFront()
{
    return run_on_simulator ? simulatorBool("wallFront") : false;
}

bool API::wallRight()
{
    return run_on_simulator ? simulatorBool("wallRight") : false;
}

void API::captureWallSample()
{
}

void API::setWallSample(int16_t left_mm, int16_t front_mm, int16_t right_mm)
{
    (void)left_mm;
    (void)front_mm;
    (void)right_mm;
}

void API::clearWallSample()
{
}

bool API::wallSample(int16_t& left_mm, int16_t& front_mm, int16_t& right_mm)
{
    (void)left_mm;
    (void)front_mm;
    (void)right_mm;
    return false;
}

void API::moveForwardHalf()
{
    if (run_on_simulator)
        simulatorResponse("moveForwardHalf");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->move(HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
    }
#endif
    // Half-cell physical moves do not advance the logical maze cell. Diagonal
    // sequences use GMF/GFM when the virtual mouse should move to the next cell.
}

bool API::move_mm(float distance_mm)
{
    if (run_on_simulator)
        return true;
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return false;
    LOG_INFO("MOTION move_mm: distance_mm=" + fixed1(distance_mm) +
             " speed_mmps=" + fixed1(ROBOT_MAX_SEARCH_SPEED_MMPS) +
             " accel_mmps2=" + fixed1(ROBOT_BASE_ACCEL_MMPS2));
    robot_->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    waitForMotion();
    return !haltRequested();
#else
    return true;
#endif
}

bool API::start_center()
{
    return move_mm(START_CENTER_DISTANCE_MM);
}

bool API::center_from_wall_check()
{
    if (!move_mm(WALL_CHECK_TO_CENTER_MM))
        return false;
    clear_search_move();
    return true;
}

bool API::search_start_from_wall_check()
{
    if (run_on_simulator)
    {
        simulatorResponse("moveForward");
        mouse_->moveForward(1);
        return true;
    }
#ifndef SIMULATOR_BUILD
    // Single-cell forward: capture wall snapshot at the wall-check position,
    // then keep moving toward the cell center. Mirrors mazerunner-core's
    // sensor-triggered approach: act on the live ToF reading mid-cell rather
    // than waiting for the profile to finish.
    if (robot_ == nullptr)
        return false;

    constexpr float kStartWallCheckPositionMm = CELL_SIZE_MM;
    const float     total_mm                  = CELL_SIZE_MM + WALL_CHECK_TO_CENTER_MM;
    LOG_INFO("MOTION search_start_from_wall_check: total_mm=" + fixed1(total_mm) +
             " latch_at_mm=" + fixed1(kStartWallCheckPositionMm) +
             " speed_mmps=" + fixed1(ROBOT_MAX_SEARCH_SPEED_MMPS) +
             " accel_mmps2=" + fixed1(ROBOT_BASE_ACCEL_MMPS2));
    robot_->move(total_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);

    while (!robot_->move_finished() && robot_->position() < kStartWallCheckPositionMm)
    {
        if (haltRequested())
        {
            robot_->emergency_stop();
            return false;
        }
        sleep_ms(2);
    }

    setWallSample(static_cast<int16_t>(robot_->leftDistance()),
                  static_cast<int16_t>(robot_->frontDistance()),
                  static_cast<int16_t>(robot_->rightDistance()));
#endif
    mouse_->moveForward(1);
    return true;
}

bool API::search_advance()
{
    if (run_on_simulator)
    {
        simulatorResponse("moveForward");
        mouse_->moveForward(1);
        return true;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return false;

    const float wall_check_position_mm = robot_->position() + CENTER_TO_NEXT_WALL_CHECK_MM;
    LOG_INFO("MOTION search_advance: distance_mm=" + fixed1(CELL_SIZE_MM) +
             " latch_after_mm=" + fixed1(CENTER_TO_NEXT_WALL_CHECK_MM) + " latch_position_mm=" +
             fixed1(wall_check_position_mm) + " speed_mmps=" + fixed1(ROBOT_MAX_SEARCH_SPEED_MMPS) +
             " accel_mmps2=" + fixed1(ROBOT_BASE_ACCEL_MMPS2));
    robot_->move(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);

    while (!robot_->move_finished() && robot_->position() < wall_check_position_mm)
    {
        if (haltRequested())
        {
            robot_->emergency_stop();
            return false;
        }
        sleep_ms(2);
    }

    setWallSample(static_cast<int16_t>(robot_->leftDistance()),
                  static_cast<int16_t>(robot_->frontDistance()),
                  static_cast<int16_t>(robot_->rightDistance()));
#endif
    mouse_->moveForward(1);
    return true;
}

void API::finish_search_move()
{
#ifndef SIMULATOR_BUILD
    waitForMotion();
#endif
}

void API::clear_search_move()
{
}

void API::moveForward()
{
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->move(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
    }
#endif
    mouse_->moveForward(1);
}

void API::moveForward(int steps)
{
    // For simulator, call single-step version in a loop (mms doesn't support moveForwardN)
    if (run_on_simulator)
    {
        for (int i = 0; i < steps; i++)
        {
            simulatorResponse("moveForward");
            mouse_->moveForward(1);
        }
        return;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ != nullptr)
    {
        robot_->move(steps * CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f,
                     ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
    }
#endif
    mouse_->moveForward(steps);
}

void API::ghostMoveForward(int steps)
{
    // Ghost move only updates internal mouse position
    mouse_->moveForward(steps);
}

void API::turnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft45");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->spin_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(-1);
}

void API::turnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("turnLeft");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        LOG_INFO(
            "MOTION turnLeft90: angle_deg=-90 omega_degps=" + fixed1(ROBOT_MAX_TURN_SPEED_DEGPS) +
            " alpha_degps2=" + fixed1(ROBOT_BASE_ANGULAR_ACCEL_DEGPS2));
        robot_->spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(-2);
}

void API::turnRight45()
{
    if (run_on_simulator)
        simulatorResponse("turnRight45");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->spin_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(1);
}

void API::turnRight90()
{
    if (run_on_simulator)
        simulatorResponse("turnRight");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        LOG_INFO(
            "MOTION turnRight90: angle_deg=90 omega_degps=" + fixed1(ROBOT_MAX_TURN_SPEED_DEGPS) +
            " alpha_degps2=" + fixed1(ROBOT_BASE_ANGULAR_ACCEL_DEGPS2));
        robot_->spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(2);
}

void API::turn(int degrees)
{
    if (run_on_simulator)
        simulatorResponse("turn" + std::to_string(degrees));
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->spin_turn(static_cast<float>(degrees), ROBOT_MAX_TURN_SPEED_DEGPS,
                          ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(degrees / 45);
}

void API::move_ahead()
{
    if (run_on_simulator)
        simulatorResponse("moveForward");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        robot_->move(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        waitForMotion();
    }
#endif
    mouse_->moveForward(1);
}

void API::turn_left()
{
    turn_smooth(SS90EL);
    mouse_->turn45Steps(-2);
}

void API::turn_right()
{
    turn_smooth(SS90ER);
    mouse_->turn45Steps(2);
}

void API::turn_back()
{
    turn_IP180();
    mouse_->turn45Steps(4);
}

void API::turn_smooth(int turn_id)
{
    if (run_on_simulator)
    {
        simulatorResponse((turn_id & 1) ? "arcTurnRight90" : "arcTurnLeft90");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return;
    // Robot::turn_smooth starts simultaneous fwd + rot profiles using the
    // SMOOTH_TURN_PARAMS table — same emergent-arc shape as mazerunner.
    robot_->turn_smooth(turn_id);
    waitForMotion();
#endif
}

void API::turn_IP180()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        simulatorResponse("turnRight");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return;
    robot_->turn_IP180();
    waitForMotion();
#endif
}

void API::turn_IP90R()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnRight");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return;
    robot_->turn_IP90R();
    waitForMotion();
#endif
}

void API::turn_IP90L()
{
    if (run_on_simulator)
    {
        simulatorResponse("turnLeft");
        return;
    }
#ifndef SIMULATOR_BUILD
    if (robot_ == nullptr)
        return;
    robot_->turn_IP90L();
    waitForMotion();
#endif
}

void API::arcTurnLeft90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft90");
#ifndef SIMULATOR_BUILD
    else
        turn_smooth(SS90EL);
#endif
    mouse_->turn45Steps(-2);
}

void API::arcTurnRight90()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight90");
#ifndef SIMULATOR_BUILD
    else
        turn_smooth(SS90ER);
#endif
    mouse_->turn45Steps(2);
}

void API::arcTurnLeft45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnLeft45");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        // 45-deg arc with a 45 mm radius — same shape startArcTurn produced.
        const float radius_mm = 45.0f;
        const float arc_mm    = 45.0f * (M_PI / 180.0f) * radius_mm;
        robot_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                           ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        robot_->start_turn(-45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                           ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(-1);
}

void API::arcTurnRight45()
{
    if (run_on_simulator)
        simulatorResponse("arcTurnRight45");
#ifndef SIMULATOR_BUILD
    else if (robot_ != nullptr)
    {
        const float radius_mm = 45.0f;
        const float arc_mm    = 45.0f * (M_PI / 180.0f) * radius_mm;
        robot_->start_move(arc_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                           ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
        robot_->start_turn(45.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                           ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
        waitForMotion();
    }
#endif
    mouse_->turn45Steps(1);
}

void API::executeSequence(const std::string& sequence)
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
                move_ahead();
            else
                moveForward();
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
                LOG_ERROR("API: Treating diagonalizer token GFM as GMF");
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
                        move_ahead();
                }
                else
                {
                    moveForward(steps);
                }
                continue;
            }
        }

        LOG_ERROR("API: Unknown path token: " + token);
        return;
    }
}

void API::setWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "setWall " << x << " " << y << " " << dir << '\n';

    Cell* cell = mouse_->cellAt(x, y);
    if (cell)
    {
        mouse_->setWallNESW(cell, dir[0]);
        if (dir.size() > 1)
            mouse_->setWallNESW(cell, dir[1]);
    }
}

void API::clearWall(int x, int y, const std::string& dir)
{
    if (run_on_simulator)
        std::cout << "clearWall " << x << " " << y << " " << dir << '\n';
}

void API::setColor(int x, int y, char color)
{
    if (run_on_simulator)
        std::cout << "setColor " << x << " " << y << " " << color << '\n';
}

void API::setPhaseColor(char color)
{
    phase_color_ = color;
}

char API::phaseColor() const
{
    return phase_color_;
}

void API::clearColor(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearColor " << x << " " << y << '\n';
}

void API::clearAllColor()
{
    if (run_on_simulator)
        std::cout << "clearAllColor" << '\n';
}

void API::setText(int x, int y, const std::string& text)
{
    if (run_on_simulator)
        std::cout << "setText " << x << " " << y << " " << text << '\n';
}

void API::clearText(int x, int y)
{
    if (run_on_simulator)
        std::cout << "clearText " << x << " " << y << '\n';
}

void API::clearAllText()
{
    if (run_on_simulator)
        std::cout << "clearAllText" << '\n';
}

std::string API::simulatorResponse(const std::string& cmd)
{
    std::cout << cmd << '\n';
    std::string resp;
    std::getline(std::cin, resp);
    return resp;
}

bool API::simulatorBool(const std::string& cmd)
{
    return simulatorResponse(cmd) == "true";
}

void API::setUp(std::array<int, 2> start, std::vector<std::array<int, 2>> goals)
{
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

void API::printMaze()
{
    std::cout << mazeString();
}

std::string API::mazeString()
{
    std::stringstream ss;
    for (int row = mazeHeight() - 1; row >= 0; row--)
    {
        ss << printMazeRow(row) << '\n';
    }
    return ss.str();
}

std::string API::printMazeRow(int row)
{
    std::stringstream ss;
    for (int col = 0; col < mazeWidth(); col++)
    {
        Cell* cell = mouse_->cellAt(col, row);
        if (cell == nullptr)
        {
            ss << "?";
            continue;
        }

        ss << (cell->explored() ? 'X' : '.');
    }
    return ss.str();
}
