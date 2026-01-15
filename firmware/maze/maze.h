#ifndef MAZE_MAZE_H
#define MAZE_MAZE_H

#include <string>
#include <vector>

/**
 * @brief Represents a single cell in the maze
 *
 * Tracks wall presence, exploration status, and pathfinding state.
 * NOTE: Phase 4 will convert this to Maze::Cell nested struct.
 */
class Cell
{
  public:
    Cell(int x, int y);

    int  x() const;
    int  y() const;
    bool explored() const;
    bool hasWall(char direction) const; // 'N', 'E', 'S', 'W'
    int  wallCount() const;

    void markExplored();
    void setNeighbor(Cell* cell, char direction);
    void setWall(char direction);

    static bool equal(Cell* c1, Cell* c2);

    // Pathfinding state (used by flood fill, A*)
    Cell* parent;
    bool  processed;
    void  clearPathfindingState();

  private:
    void addWall(char direction);

    int   x_, y_;
    Cell* north_;
    Cell* east_;
    Cell* south_;
    Cell* west_;
    bool  wall_north_, wall_east_, wall_south_, wall_west_;
    bool  explored_;
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

    int  width() const;
    int  height() const;
    void printASCII();

  private:
    void createCells();
    void linkNeighbors();

    std::string topEdge();
    std::string rowString(int row);

    std::vector<std::vector<Cell*>> cells_;
};

#endif
