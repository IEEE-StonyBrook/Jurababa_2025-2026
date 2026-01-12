#ifndef MAZE_MOUSE_H
#define MAZE_MOUSE_H

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "common/log.h"
#include "maze/maze.h"

/**
 * @brief Virtual mouse position tracker in the maze
 *
 * Tracks the robot's logical position within the maze grid, independent
 * of physical coordinates. Used by pathfinding algorithms.
 */
class Mouse
{
  public:
    // === Construction ===
    Mouse(std::array<int, 2> start_pos, std::string start_dir,
          std::vector<std::array<int, 2>> goals, Maze* maze);

    // === Position & Movement ===
    Cell* currentCell();
    std::string currentDirection();
    std::array<int, 2> currentDirectionArray();
    void setPosition(Cell* cell);
    void moveForward(int cells);
    bool canMoveBetween(Cell* from, Cell* to, bool diagonals = false);

    // === Direction & Turning ===
    void turn45Steps(int half_steps_right);
    std::vector<std::array<int, 2>> possibleDirections();
    std::string directionAsString(const std::array<int, 2>& dir) const;
    std::string directionLeft() const;
    std::string directionRight() const;
    int findDirectionIndex(const std::string& dir) const;

    // === Wall Detection ===
    void setWallLFR(char direction);
    void setWallNESW(Cell* cell, char direction);

    // === Goal Management ===
    bool isGoal(Cell* cell);
    void setGoals(std::vector<std::array<int, 2>> goals);
    std::vector<std::array<int, 2>> goals();

    // === Maze Access ===
    int mazeWidth();
    int mazeHeight();
    Cell* cellAt(int x, int y);
    std::vector<Cell*> cellNeighbors(Cell* cell, bool include_diagonal = false);
    void resetPathfinding();

    // === Heuristics (Static) ===
    static double euclidean(Cell& c1, Cell& c2);
    static double octile(Cell& c1, Cell& c2);

  private:
    int directionIndex(std::string dir);
    std::string directionAfterTurn(int half_steps);

    std::array<int, 2> position_;
    std::string direction_;
    std::vector<std::array<int, 2>> goals_;
    Maze* maze_;

    const std::array<std::string, 8> dir_names_ = {"n", "ne", "e", "se", "s", "sw", "w", "nw"};
    const std::map<std::string, std::array<int, 2>> dir_offsets_ = {
        {"n", {0, 1}},  {"ne", {1, 1}},   {"e", {1, 0}},  {"se", {1, -1}},
        {"s", {0, -1}}, {"sw", {-1, -1}}, {"w", {-1, 0}}, {"nw", {-1, 1}}};
};

#endif
