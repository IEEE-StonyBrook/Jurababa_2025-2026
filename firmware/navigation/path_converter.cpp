#include "navigation/path_converter.h"

#include "common/log.h"
#include "maze/maze.h"

Cell* PathConverter::current_cell_ = nullptr;
std::array<int, 2> PathConverter::current_heading_ = {0, 1};

std::string PathConverter::buildLFR(Cell* start_cell,
                                    std::array<int, 2> start_heading,
                                    const std::vector<Cell*>& path)
{
    std::string lfr;
    if (path.empty())
        return lfr;

    current_cell_ = start_cell;
    current_heading_ = start_heading;

    for (Cell* next : path)
    {
        std::array<int, 2> needed = {
            next->x() - current_cell_->x(),
            next->y() - current_cell_->y()
        };
        lfr += movesForHeadingChange(current_heading_, needed);
        current_cell_ = next;
    }

    return lfr;
}

std::string PathConverter::movesForHeadingChange(std::array<int, 2> from,
                                                 std::array<int, 2> to)
{
    // Cardinal only - no diagonal starting position
    if (from[0] != 0 && from[1] != 0)
    {
        LOG_ERROR("PathConverter: Mouse on diagonal! Not allowed.");
        return "";
    }
    if (to[0] == 0 && to[1] == 0)
    {
        LOG_ERROR("PathConverter: Invalid 'to' heading.");
        return "";
    }

    // Same direction - just move forward
    if (from[0] == to[0] && from[1] == to[1])
    {
        return "F#";
    }

    // Cardinal target (N, E, S, W)
    if (to[0] == 0 || to[1] == 0)
    {
        current_heading_ = to;
        return cardinalMoves(from, to);
    }

    // Diagonal target - decompose into vertical then horizontal
    std::array<int, 2> vertical = {0, to[1]};
    std::array<int, 2> horizontal = {to[0], 0};
    std::string moves;
    moves += cardinalMoves(from, vertical);
    moves += cardinalMoves(vertical, horizontal);
    current_heading_ = horizontal;
    return moves;
}

std::string PathConverter::cardinalMoves(std::array<int, 2> from,
                                         std::array<int, 2> to)
{
    const std::vector<std::array<int, 2>> dirs = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}  // N, E, S, W
    };

    int from_idx = findDirectionIndex(dirs, from);
    int to_idx = findDirectionIndex(dirs, to);

    if (from_idx == -1 || to_idx == -1)
    {
        LOG_ERROR("PathConverter: Not 4 cardinal directions!");
        return "";
    }

    // Calculate turns needed (positive = right turns)
    int turns = ((to_idx - from_idx) % 4 + 4) % 4;

    switch (turns)
    {
        case 0: return "F#";
        case 1: return "R#F#";
        case 2: return "R#R#F#";
        case 3: return "L#F#";
    }

    return "";
}

int PathConverter::findDirectionIndex(const std::vector<std::array<int, 2>>& dirs,
                                      std::array<int, 2> dir)
{
    for (size_t i = 0; i < dirs.size(); i++)
    {
        if (dirs[i][0] == dir[0] && dirs[i][1] == dir[1])
            return static_cast<int>(i);
    }
    return -1;
}
