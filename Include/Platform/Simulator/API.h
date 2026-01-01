#ifndef APISIMULATOR_H
#define APISIMULATOR_H

#include <string>

#include "../IAPIInterface.h"
#include "../../Maze/InternalMouse.h"

class API_SIMULATOR : public IAPIInterface
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
    int mazeWidth() override;

    /**
     * Gets the height of the maze.
     *
     * @return The height of the maze as an integer.
     */
    int mazeHeight() override;

    /**
     * Checks if there is a wall to the left of the mouse.
     *
     * @return True if there is a wall to the left, false otherwise.
     */
    bool wallLeft() override;

    /**
     * Checks if there is a wall in front of the mouse.
     *
     * @return True if there is a wall in front, false otherwise.
     */
    bool wallFront() override;

    /**
     * Checks if there is a wall to the right of the mouse.
     *
     * @return True if there is a wall to the right, false otherwise.
     */
    bool wallRight() override;

    /**
     * Moves the mouse forward by half a cell.
     */
    void moveForwardHalf() override;

    /**
     * Moves the mouse forward by one cell.
     */
    void moveForward() override;

    /**
     * Moves the mouse forward by a specified number of steps.
     *
     * @param steps The number of steps to move forward.
     */
    void moveForward(int steps) override;

    /**
     * Turns the mouse left by 45 degrees.
     */
    void turnLeft45() override;

    /**
     * Turns the mouse left by 90 degrees.
     */
    void turnLeft90() override;

    /**
     * Turns the mouse right by 45 degrees.
     */
    void turnRight45() override;

    /**
     * Turns the mouse right by 90 degrees.
     */
    void turnRight90() override;

    /**
     * Turns the mouse by a specified angle divisible by 45 degrees.
     *
     * @param degreesDivisibleBy45 The angle to turn, must be divisible by 45.
     */
    void turn(int degreesDivisibleBy45) override;

    /**
     * Sets a wall at the specified coordinates and direction.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param direction The direction of the wall (e.g., "N", "E", "S", "W").
     */
    void setWall(int x, int y, const std::string& direction) override;

    /**
     * Clears a wall at the specified coordinates and direction.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param direction The direction of the wall (e.g., "N", "E", "S", "W").
     */
    void clearWall(int x, int y, const std::string& direction) override;

    /**
     * Sets the color of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param color The color to set (e.g., 'R', 'G', 'B').
     */
    void setColor(int x, int y, char color) override;

    /**
     * Clears the color of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     */
    void clearColor(int x, int y) override;

    /**
     * Clears the color of all cells in the maze.
     */
    void clearAllColor() override;

    /**
     * Sets the text of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     * @param text The text to set.
     */
    void setText(int x, int y, const std::string& text) override;

    /**
     * Clears the text of a cell at the specified coordinates.
     *
     * @param x The x-coordinate of the cell.
     * @param y The y-coordinate of the cell.
     */
    void clearText(int x, int y) override;

    /**
     * Clears the text of all cells in the maze.
     */
    void clearAllText() override;

    void setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells) override;
    void printMaze() override;
    void executeSequence(const std::string& seq) override;

    bool runOnSimulator;

  private:
  InternalMouse* internalMouse;

    std::string getSimulatorResponse(std::string commandUsed);
    int         getSimulatorIntegerResponse(std::string commandUsed);
    bool        getSimulatorBoolResponse(std::string commandUsed);

    std::string printMazeRow(int row);
};
#endif