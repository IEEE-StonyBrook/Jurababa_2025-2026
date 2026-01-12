#ifndef APP_API_H
#define APP_API_H

#include <array>
#include <string>
#include <vector>

class Mouse;
class Drivetrain;

/**
 * @brief High-level maze navigation API
 *
 * Provides movement commands, wall sensing, and maze visualization.
 * Bridges between navigation algorithms and hardware control.
 */
class API
{
  public:
    explicit API(Mouse* mouse);

    // Maze dimensions
    int mazeWidth();
    int mazeHeight();

    // Wall queries
    bool wallLeft();
    bool wallFront();
    bool wallRight();

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

    bool run_on_simulator = false;

  private:
    std::string simulatorResponse(const std::string& cmd);
    int simulatorInt(const std::string& cmd);
    bool simulatorBool(const std::string& cmd);
    std::string printMazeRow(int row);

    Mouse* mouse_;
    char phase_color_ = 'y';
};

#endif
