#include "maze/maze.h"

#include <sstream>

// ============================================================
// Cell Implementation
// ============================================================

Cell::Cell(int x, int y)
    : x_(x), y_(y), north_(nullptr), east_(nullptr), south_(nullptr), west_(nullptr),
      explored_(false), parent(nullptr), processed(false)
{
}

int Cell::x() const
{
    return x_;
}
int Cell::y() const
{
    return y_;
}
bool Cell::explored() const
{
    return explored_;
}

bool Cell::hasWall(char direction) const
{
    return wallState(direction) == WALL;
}

bool Cell::isExit(char direction, MazeMask mask) const
{
    return (wallState(direction) & mask) == EXIT;
}

bool Cell::hasUnknownWalls() const
{
    return walls_.north == UNKNOWN || walls_.east == UNKNOWN || walls_.south == UNKNOWN ||
           walls_.west == UNKNOWN;
}

WallState Cell::wallState(char direction) const
{
    switch (direction)
    {
        case 'N':
        case 'n':
            return walls_.north;
        case 'E':
        case 'e':
            return walls_.east;
        case 'S':
        case 's':
            return walls_.south;
        case 'W':
        case 'w':
            return walls_.west;
        default:
            return WALL;
    }
}

WallInfo Cell::walls() const
{
    return walls_;
}

int Cell::wallCount() const
{
    int count = 0;
    if (walls_.north == WALL)
        count++;
    if (walls_.east == WALL)
        count++;
    if (walls_.south == WALL)
        count++;
    if (walls_.west == WALL)
        count++;
    return count;
}

void Cell::markExplored()
{
    explored_ = true;
}

void Cell::setNeighbor(Cell* cell, char direction)
{
    switch (direction)
    {
        case 'N':
        case 'n':
            north_ = cell;
            break;
        case 'E':
        case 'e':
            east_ = cell;
            break;
        case 'S':
        case 's':
            south_ = cell;
            break;
        case 'W':
        case 'w':
            west_ = cell;
            break;
    }
}

void Cell::setWall(char direction)
{
    setWallStateLocal(direction, WALL);

    // Set shared wall on neighbor cell (walls are bidirectional)
    switch (direction)
    {
        case 'N':
        case 'n':
            if (north_)
                north_->setWallStateLocal('S', WALL);
            break;
        case 'E':
        case 'e':
            if (east_)
                east_->setWallStateLocal('W', WALL);
            break;
        case 'S':
        case 's':
            if (south_)
                south_->setWallStateLocal('N', WALL);
            break;
        case 'W':
        case 'w':
            if (west_)
                west_->setWallStateLocal('E', WALL);
            break;
    }
}

void Cell::setExit(char direction)
{
    setWallStateLocal(direction, EXIT);

    switch (direction)
    {
        case 'N':
        case 'n':
            if (north_)
                north_->setWallStateLocal('S', EXIT);
            break;
        case 'E':
        case 'e':
            if (east_)
                east_->setWallStateLocal('W', EXIT);
            break;
        case 'S':
        case 's':
            if (south_)
                south_->setWallStateLocal('N', EXIT);
            break;
        case 'W':
        case 'w':
            if (west_)
                west_->setWallStateLocal('E', EXIT);
            break;
    }
}

void Cell::updateWallState(char direction, WallState state)
{
    if (!shouldUpdate(direction))
        return;

    if (state == WALL)
        setWall(direction);
    else if (state == EXIT)
        setExit(direction);
    else
        setWallStateLocal(direction, state);
}

void Cell::update_wall_state(char direction, WallState state)
{
    updateWallState(direction, state);
}

void Cell::set_wall_state(char direction, WallState state)
{
    if (state == WALL)
        setWall(direction);
    else if (state == EXIT)
        setExit(direction);
    else
        setWallStateLocal(direction, state);
}

void Cell::reset()
{
    walls_    = {};
    explored_ = false;
    clearPathfindingState();
}

void Cell::setWallStateLocal(char direction, WallState state)
{
    switch (direction)
    {
        case 'N':
        case 'n':
            walls_.north = state;
            break;
        case 'E':
        case 'e':
            walls_.east = state;
            break;
        case 'S':
        case 's':
            walls_.south = state;
            break;
        case 'W':
        case 'w':
            walls_.west = state;
            break;
    }
}

bool Cell::shouldUpdate(char direction) const
{
    return wallState(direction) == UNKNOWN;
}

bool Cell::equal(Cell* c1, Cell* c2)
{
    if (c1 == nullptr || c2 == nullptr)
        return false;
    return (c1->x_ == c2->x_) && (c1->y_ == c2->y_);
}

void Cell::clearPathfindingState()
{
    parent    = nullptr;
    processed = false;
}

// ============================================================
// Maze Implementation
// ============================================================

Maze::Maze(int rows, int cols)
{
    cells_ = std::vector<std::vector<Cell*>>(cols, std::vector<Cell*>(rows));
    createCells();
    linkNeighbors();
}

Maze::~Maze()
{
    for (size_t col = 0; col < cells_.size(); col++)
    {
        for (size_t row = 0; row < cells_[0].size(); row++)
        {
            delete cells_[col][row];
        }
    }
}

void Maze::createCells()
{
    for (size_t col = 0; col < cells_.size(); col++)
    {
        for (size_t row = 0; row < cells_[0].size(); row++)
        {
            cells_[col][row] = new Cell(col, row);
        }
    }
}

void Maze::linkNeighbors()
{
    int cols = cells_.size();
    int rows = cells_[0].size();

    for (int col = 0; col < cols; col++)
    {
        for (int row = 0; row < rows; row++)
        {
            Cell* current = cells_[col][row];

            if (row < rows - 1)
                current->setNeighbor(cells_[col][row + 1], 'N');
            if (col < cols - 1)
                current->setNeighbor(cells_[col + 1][row], 'E');
            if (row > 0)
                current->setNeighbor(cells_[col][row - 1], 'S');
            if (col > 0)
                current->setNeighbor(cells_[col - 1][row], 'W');
        }
    }
}

Cell* Maze::cell(int x, int y)
{
    return cells_[x][y];
}

std::vector<Cell*> Maze::neighbors(Cell* c, bool include_diagonal)
{
    std::vector<Cell*> result;
    int                x = c->x();
    int                y = c->y();
    int                w = width();
    int                h = height();

    // Cardinal directions
    if (x > 0)
        result.push_back(cells_[x - 1][y]);
    if (x < w - 1)
        result.push_back(cells_[x + 1][y]);
    if (y > 0)
        result.push_back(cells_[x][y - 1]);
    if (y < h - 1)
        result.push_back(cells_[x][y + 1]);

    // Diagonal directions
    if (include_diagonal)
    {
        if (x > 0 && y > 0)
            result.push_back(cells_[x - 1][y - 1]);
        if (x > 0 && y < h - 1)
            result.push_back(cells_[x - 1][y + 1]);
        if (x < w - 1 && y > 0)
            result.push_back(cells_[x + 1][y - 1]);
        if (x < w - 1 && y < h - 1)
            result.push_back(cells_[x + 1][y + 1]);
    }

    return result;
}

int Maze::width() const
{
    return cells_.size();
}
int Maze::height() const
{
    return cells_[0].size();
}

void Maze::reset()
{
    for (size_t col = 0; col < cells_.size(); col++)
    {
        for (size_t row = 0; row < cells_[0].size(); row++)
        {
            cells_[col][row]->reset();
        }
    }
    mask_ = MASK_OPEN;
}

void Maze::setMask(MazeMask mask)
{
    mask_ = mask;
}

MazeMask Maze::mask() const
{
    return mask_;
}

void Maze::printASCII()
{
    std::stringstream ss;
    ss << "ASCII Maze:\n";
    ss << topEdge();

    for (int row = cells_[0].size() - 1; row >= 0; row--)
    {
        ss << rowString(row);
    }
}

std::string Maze::topEdge()
{
    std::stringstream ss;
    for (size_t col = 0; col < cells_.size(); col++)
    {
        ss << "+---";
    }
    ss << "+\n";
    return ss.str();
}

std::string Maze::rowString(int row)
{
    std::stringstream vertical, horizontal;

    vertical << "|";
    for (size_t col = 0; col < cells_.size(); col++)
    {
        Cell* c = cells_[col][row];
        if (c->hasWall('E'))
            vertical << "   |";
        else
            vertical << "    ";

        if (c->hasWall('S'))
            horizontal << "+---";
        else
            horizontal << "+   ";
    }

    std::stringstream result;
    result << vertical.str() << "\n" << horizontal.str();
    return result.str();
}
