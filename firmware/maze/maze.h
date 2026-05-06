#ifndef MAZE_MAZE_H
#define MAZE_MAZE_H

#include <string>
#include <vector>

enum WallState
{
    EXIT    = 0,
    WALL    = 1,
    UNKNOWN = 2,
    VIRTUAL = 3,
};

enum MazeMask
{
    MASK_OPEN   = 0x01,
    MASK_CLOSED = 0x03,
};

struct WallInfo
{
    WallState north = UNKNOWN;
    WallState east  = UNKNOWN;
    WallState south = UNKNOWN;
    WallState west  = UNKNOWN;
};

/**
 * @brief Represents a single cell in the maze
 *
 * Tracks wall presence, exploration status, and pathfinding state.
 */
class Cell
{
  public:
    Cell(int x, int y);

    int       x() const;
    int       y() const;
    bool      explored() const;
    bool      hasWall(char direction) const; // 'N', 'E', 'S', 'W'
    bool      isExit(char direction, MazeMask mask = MASK_OPEN) const;
    bool      hasUnknownWalls() const;
    WallState wallState(char direction) const;
    WallInfo  walls() const;
    int       wallCount() const;

    void markExplored();
    void setNeighbor(Cell* cell, char direction);
    void setWall(char direction);
    void setExit(char direction);
    void updateWallState(char direction, WallState state);
    void update_wall_state(char direction, WallState state);
    void set_wall_state(char direction, WallState state);
    void reset();

    static bool equal(Cell* c1, Cell* c2);

    // Pathfinding state (used by flood fill, A*)
    Cell* parent;
    bool  processed;
    void  clearPathfindingState();

  private:
    void setWallStateLocal(char direction, WallState state);
    bool shouldUpdate(char direction) const;

    int      x_, y_;
    Cell*    north_;
    Cell*    east_;
    Cell*    south_;
    Cell*    west_;
    WallInfo walls_;
    bool     explored_;
};

/**
 * @brief Maze graph with wall tracking and neighbor queries
 */
class Maze
{
  public:
    Maze(int rows, int cols);
    ~Maze();

    Cell*              cell(int x, int y);
    std::vector<Cell*> neighbors(Cell* cell, bool include_diagonal = false);

    int      width() const;
    int      height() const;
    void     reset();
    void     printASCII();
    void     setMask(MazeMask mask);
    MazeMask mask() const;

  private:
    void createCells();
    void linkNeighbors();

    std::string topEdge();
    std::string rowString(int row);

    std::vector<std::vector<Cell*>> cells_;
    MazeMask                        mask_ = MASK_OPEN;
};

#endif
