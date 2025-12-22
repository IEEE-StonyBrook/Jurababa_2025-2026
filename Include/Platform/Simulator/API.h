#ifndef APISIMULATOR_H
#define APISIMULATOR_H

#include <string>

#include "../../Maze/InternalMouse.h"

class API_SIMULATOR
{
  public:
    /**
     * Constructs an API object for simulation.
     *
     * @param internalMouse Pointer to the internal mouse object.
     * @param runOnSimulator Boolean indicating if the API should run on a
     * simulator.
     */
    API_SIMULATOR(InternalMouse* internalMouse, bool runOnSimulator);

    /**
     * Gets the width of the maze.
     *
     * @return The width of the maze as an integer.
     */
    int mazeWidth();

    /**
     * Gets the height of the maze.
     *
     * @return The height of the maze as an integer.
     */
    int mazeHeight();

    /**
     * Checks if there is a wall to the left of the mouse.
     *
     * @return True if there is a wall to the left, false otherwise.
     */
    bool wallLeft();

    /**
     * Checks if there is a wall in front of the mouse.
     *
     * @return True if there is a wall in front, false otherwise.
     */
    bool wallFront();

    /**
     * Checks if there is a wall to the right of the mouse.
     *
     * @return True if there is a wall to the right, false otherwise.
     */
    bool wallRight();

    /**
     * Moves the mouse forward by half a cell.
     */
    void moveForwardHalf();

    /**
     * Moves the mouse forward by one cell.
     */
    void moveForward();

    /**
     * Moves the mouse forward by a specified number of steps.
     *
     * @param steps The number of steps to move forward.
     */
    void moveForward(int steps);

    /**
     * Turns the mouse left by 45 degrees.
     */
    void turnLeft45();

    /**
     * Turns the mouse left by 90 degrees.
     */
    void turnLeft90();

    /**
     * Turns the mouse right by 45 degrees.
     */
    void turnRight45();

    /**
     * Turns the mouse right by 90 degrees.
     */
    void turnRight90();

    /**
     * Turns the mouse by a specified angle divisible by 45 degrees.
     *
     * @param degreesDivisibleBy45 The angle to turn, must be divisible by 45.
     */
    void turn(int degreesDivisibleBy45);

    /**
     * Sets a wall at the specified coordinates and direction.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param direction The direction of the wall (e.g., "N", "E", "S", "W").
     */
    void setWall(int x, int y, const std::string& direction);

    /**
     * Clears a wall at the specified coordinates and direction.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param direction The direction of the wall (e.g., "N", "E", "S", "W").
     */
    void clearWall(int x, int y, const std::string& direction);

    /**
     * Sets the color of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param color The color to set (e.g., 'R', 'G', 'B').
     */
    void setColor(int x, int y, char color);

    /**
     * Clears the color of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     */
    void clearColor(int x, int y);

    /**
     * Clears the color of all cells in the maze.
     */
    void clearAllColor();

    /**
     * Sets the text of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param text The text to set.
     */
    void setText(int x, int y, const std::string& text);

    /**
     * Clears the text of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     */
    void clearText(int x, int y);

    /**
     * Clears the text of all cells in the maze.
     */
    void clearAllText();

    void setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells);
    void printMaze();
    bool runOnSimulator;

  void executeSequence(const std::string& seq);
 private:
  InternalMouse* internalMouse;

    std::string getSimulatorResponse(std::string commandUsed);
    int         getSimulatorIntegerResponse(std::string commandUsed);
    bool        getSimulatorBoolResponse(std::string commandUsed);

    std::string printMazeRow(int row);
};
#endif