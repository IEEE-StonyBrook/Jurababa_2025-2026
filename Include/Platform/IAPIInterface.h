#ifndef IAPIINTERFACE_H
#define IAPIINTERFACE_H

#include <array>
#include <string>
#include <vector>

/**
 * Abstract interface for maze navigation API.
 * Provides platform-agnostic methods for maze querying, robot movement,
 * and visualization that work on both physical hardware (Pico) and simulator.
 */
class IAPIInterface
{
  public:
    virtual ~IAPIInterface() = default;

    // Maze dimensions.
    virtual int mazeWidth()  = 0;
    virtual int mazeHeight() = 0;

    // Wall queries.
    virtual bool wallLeft()  = 0;
    virtual bool wallFront() = 0;
    virtual bool wallRight() = 0;

    // Movement.
    virtual void moveForwardHalf()           = 0;
    virtual void moveForward()               = 0;
    virtual void moveForward(int steps)      = 0;
    virtual void ghostMoveForward(int steps) = 0;

    virtual void turnLeft45()      = 0;
    virtual void turnLeft90()      = 0;
    virtual void turnRight45()     = 0;
    virtual void turnRight90()     = 0;
    virtual void turn(int degrees) = 0;

    // Arc turns (smooth turns combining forward motion + rotation).
    virtual void arcTurnLeft90()  = 0;
    virtual void arcTurnRight90() = 0;
    virtual void arcTurnLeft45()  = 0;
    virtual void arcTurnRight45() = 0;

    // Execute command sequence like "L90#F5#R45".
    virtual void executeSequence(const std::string& sequence) = 0;

    // Maze walls.
    virtual void setWall(int x, int y, const std::string& dir)   = 0;
    virtual void clearWall(int x, int y, const std::string& dir) = 0;

    // Cell coloring and labels.
    virtual void setColor(int x, int y, char color)             = 0;
    virtual void clearColor(int x, int y)                       = 0;
    virtual void clearAllColor()                                = 0;
    virtual void setText(int x, int y, const std::string& text) = 0;
    virtual void clearText(int x, int y)                        = 0;
    virtual void clearAllText()                                 = 0;

    // Phase tracking for visualization.
    virtual void setPhaseColor(char color) = 0;

    // Setup and print.
    virtual void setUp(std::array<int, 2> startCell, std::vector<std::array<int, 2>> goalCells) = 0;
    virtual void printMaze()                                                                    = 0;
};

#endif
