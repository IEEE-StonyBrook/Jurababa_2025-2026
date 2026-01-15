#include "maze/mouse.h"

#include <algorithm>
#include <cmath>

// =============================================================================
// Construction
// =============================================================================

Mouse::Mouse(std::array<int, 2> start_pos, std::string start_dir,
             std::vector<std::array<int, 2>> goals, Maze* maze)
    : position_(start_pos), direction_(start_dir), goals_(goals), maze_(maze)
{
}

// =============================================================================
// Position & Movement
// =============================================================================

Cell* Mouse::currentCell()
{
    return maze_->cell(position_[0], position_[1]);
}

std::string Mouse::currentDirection()
{
    return direction_;
}

std::array<int, 2> Mouse::currentDirectionArray()
{
    return dir_offsets_.at(direction_);
}

void Mouse::setPosition(Cell* cell)
{
    position_ = {cell->x(), cell->y()};
}

void Mouse::moveForward(int cells)
{
    std::array<int, 2> offset = dir_offsets_.at(direction_);
    position_[0] += offset[0] * cells;
    position_[1] += offset[1] * cells;
}

bool Mouse::canMoveBetween(Cell* from, Cell* to, bool diagonals)
{
    int dx = to->x() - from->x();
    int dy = to->y() - from->y();

    // Cardinal directions
    if (dx == 0 && dy == 1)
        return !from->hasWall('N');
    if (dx == 1 && dy == 0)
        return !from->hasWall('E');
    if (dx == 0 && dy == -1)
        return !from->hasWall('S');
    if (dx == -1 && dy == 0)
        return !from->hasWall('W');

    // Diagonals
    if (diagonals)
    {
        int x = from->x(), y = from->y();
        if (dx == 1 && dy == 1)
            return !from->hasWall('N') && !maze_->cell(x, y + 1)->hasWall('E');
        if (dx == -1 && dy == 1)
            return !from->hasWall('N') && !maze_->cell(x, y + 1)->hasWall('W');
        if (dx == 1 && dy == -1)
            return !from->hasWall('S') && !maze_->cell(x, y - 1)->hasWall('E');
        if (dx == -1 && dy == -1)
            return !from->hasWall('S') && !maze_->cell(x, y - 1)->hasWall('W');
    }

    return false;
}

// =============================================================================
// Direction & Turning
// =============================================================================

void Mouse::turn45Steps(int half_steps_right)
{
    direction_ = directionAfterTurn(half_steps_right);
}

std::vector<std::array<int, 2>> Mouse::possibleDirections()
{
    std::vector<std::array<int, 2>> result;
    for (auto& pair : dir_offsets_)
    {
        result.push_back(pair.second);
    }
    return result;
}

std::string Mouse::directionAsString(const std::array<int, 2>& dir) const
{
    const std::vector<std::array<int, 2>> offsets = {{0, 1},  {1, 1},   {1, 0},  {1, -1},
                                                     {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        if (offsets[i][0] == dir[0] && offsets[i][1] == dir[1])
            return dir_names_[i];
    }
    return "invalid";
}

std::string Mouse::directionLeft() const
{
    int idx = 0;
    for (size_t i = 0; i < dir_names_.size(); i++)
    {
        if (dir_names_[i] == direction_)
        {
            idx = i;
            break;
        }
    }
    int left_idx = (idx + 6) % 8;
    return dir_names_[left_idx];
}

std::string Mouse::directionRight() const
{
    int idx = 0;
    for (size_t i = 0; i < dir_names_.size(); i++)
    {
        if (dir_names_[i] == direction_)
        {
            idx = i;
            break;
        }
    }
    int right_idx = (idx + 2) % 8;
    return dir_names_[right_idx];
}

int Mouse::findDirectionIndex(const std::string& dir) const
{
    for (size_t i = 0; i < dir_names_.size(); i++)
    {
        if (dir_names_[i] == dir)
            return i;
    }
    return -1;
}

// =============================================================================
// Wall Detection
// =============================================================================

void Mouse::setWallLFR(char direction)
{
    int  half_steps = 0;
    char lower      = tolower(direction);
    if (lower == 'l')
        half_steps = -2;
    if (lower == 'r')
        half_steps = 2;

    std::string        dir_str = directionAfterTurn(half_steps);
    std::array<int, 2> offset  = dir_offsets_.at(dir_str);
    Cell*              cell    = currentCell();

    if (offset[1] == 1)
        cell->setWall('N');
    if (offset[0] == 1)
        cell->setWall('E');
    if (offset[1] == -1)
        cell->setWall('S');
    if (offset[0] == -1)
        cell->setWall('W');
}

void Mouse::setWallNESW(Cell* cell, char direction)
{
    cell->setWall(direction);
}

// =============================================================================
// Goal Management
// =============================================================================

bool Mouse::isGoal(Cell* cell)
{
    for (auto& goal : goals_)
    {
        if (Cell::equal(cell, maze_->cell(goal[0], goal[1])))
            return true;
    }
    return false;
}

void Mouse::setGoals(std::vector<std::array<int, 2>> goals)
{
    goals_ = goals;
}

std::vector<std::array<int, 2>> Mouse::goals()
{
    return goals_;
}

// =============================================================================
// Maze Access
// =============================================================================

int Mouse::mazeWidth()
{
    return maze_->width();
}

int Mouse::mazeHeight()
{
    return maze_->height();
}

Cell* Mouse::cellAt(int x, int y)
{
    return maze_->cell(x, y);
}

std::vector<Cell*> Mouse::cellNeighbors(Cell* cell, bool include_diagonal)
{
    return maze_->neighbors(cell, include_diagonal);
}

void Mouse::resetPathfinding()
{
    for (int col = 0; col < mazeWidth(); col++)
    {
        for (int row = 0; row < mazeHeight(); row++)
        {
            maze_->cell(col, row)->clearPathfindingState();
        }
    }
}

// =============================================================================
// Heuristics (Static)
// =============================================================================

double Mouse::euclidean(Cell& c1, Cell& c2)
{
    return std::sqrt(std::pow(c1.x() - c2.x(), 2) + std::pow(c1.y() - c2.y(), 2));
}

double Mouse::octile(Cell& c1, Cell& c2)
{
    int dx = std::abs(c1.x() - c2.x());
    int dy = std::abs(c1.y() - c2.y());
    return (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
}

// =============================================================================
// Private Helpers
// =============================================================================

int Mouse::directionIndex(std::string dir)
{
    for (size_t i = 0; i < dir_names_.size(); i++)
    {
        if (dir_names_[i] == dir)
            return i;
    }
    return -1;
}

std::string Mouse::directionAfterTurn(int half_steps)
{
    if (half_steps == 0)
        return direction_;

    int idx     = directionIndex(direction_);
    int new_idx = ((idx + half_steps) % 8 + 8) % 8;
    return dir_names_[new_idx];
}
