#include "maze/maze.h"

#include <sstream>

// ============================================================
// Cell Implementation
// ============================================================

Cell::Cell(int x, int y)
    : x_(x), y_(y), north_(nullptr), east_(nullptr), south_(nullptr), west_(nullptr),
      wall_north_(false), wall_east_(false), wall_south_(false), wall_west_(false),
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
    switch (direction)
    {
        case 'N':
        case 'n':
            return wall_north_;
        case 'E':
        case 'e':
            return wall_east_;
        case 'S':
        case 's':
            return wall_south_;
        case 'W':
        case 'w':
            return wall_west_;
        default:
            return false;
    }
}

int Cell::wallCount() const
{
    int count = 0;
    if (wall_north_)
        count++;
    if (wall_east_)
        count++;
    if (wall_south_)
        count++;
    if (wall_west_)
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
    addWall(direction);

    // Set shared wall on neighbor cell (walls are bidirectional)
    switch (direction)
    {
        case 'N':
        case 'n':
            if (north_)
                north_->addWall('S');
            break;
        case 'E':
        case 'e':
            if (east_)
                east_->addWall('W');
            break;
        case 'S':
        case 's':
            if (south_)
                south_->addWall('N');
            break;
        case 'W':
        case 'w':
            if (west_)
                west_->addWall('E');
            break;
    }
}

void Cell::addWall(char direction)
{
    switch (direction)
    {
        case 'N':
        case 'n':
            wall_north_ = true;
            break;
        case 'E':
        case 'e':
            wall_east_ = true;
            break;
        case 'S':
        case 's':
            wall_south_ = true;
            break;
        case 'W':
        case 'w':
            wall_west_ = true;
            break;
    }
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
