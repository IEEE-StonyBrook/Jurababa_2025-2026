#include "navigation/a_star.h"
#include "navigation/path_converter.h"

AStar::AStar(Mouse* mouse) : mouse_(mouse) {}

std::string AStar::pathTo(std::vector<std::array<int, 2>> goals,
                          bool diagonals, bool pass_goals)
{
    std::vector<Cell*> path = findBestPath(goals, diagonals, pass_goals);
    LOG_DEBUG(pathToString(path));

    std::string lfr = PathConverter::buildLFR(
        mouse_->currentCell(),
        mouse_->currentDirectionArray(),
        path);
    LOG_DEBUG(lfr);
    return lfr;
}

std::vector<Cell*> AStar::cellPath(std::vector<std::array<int, 2>> goals,
                                   bool diagonals, bool pass_goals)
{
    return findBestPath(goals, diagonals, pass_goals);
}

std::vector<Cell*> AStar::findBestPath(std::vector<std::array<int, 2>> goals,
                                       bool diagonals, bool pass_goals)
{
    std::vector<Cell*> best;
    float best_cost = std::numeric_limits<float>::infinity();

    for (auto& goal : goals)
    {
        std::vector<Cell*> path = findPathTo(
            mouse_->cellAt(goal[0], goal[1]), diagonals, pass_goals);

        if (path.empty())
            continue;

        if (total_cost_ < best_cost)
        {
            best = path;
            best_cost = total_cost_;
        }
    }

    return best;
}

struct AStarNode
{
    Cell* cell;
    float g_cost, f_cost;
    bool operator<(const AStarNode& other) const { return f_cost > other.f_cost; }
};

std::vector<Cell*> AStar::findPathTo(Cell* end, bool diagonals, bool pass_goals)
{
    Cell* start = mouse_->currentCell();
    int w = mouse_->mazeWidth();
    int h = mouse_->mazeHeight();

    std::vector<std::vector<float>> g_costs(
        w, std::vector<float>(h, std::numeric_limits<float>::infinity()));

    std::priority_queue<AStarNode> open;

    g_costs[start->x()][start->y()] = 0.0f;
    open.push({start, 0.0f, heuristic(start, end)});

    while (!open.empty())
    {
        AStarNode current = open.top();
        open.pop();

        if (current.cell == end)
        {
            total_cost_ = current.f_cost;
            return reconstructPath(start, end);
        }

        if (current.cell->processed)
            continue;
        current.cell->processed = true;

        for (Cell* neighbor : mouse_->cellNeighbors(current.cell, diagonals))
        {
            bool is_end = (neighbor == end);
            if (neighbor->processed ||
                (!pass_goals && mouse_->isGoal(neighbor) && !is_end) ||
                !mouse_->canMoveBetween(current.cell, neighbor, diagonals))
            {
                continue;
            }

            float new_g = current.g_cost + heuristic(current.cell, neighbor);
            int nx = neighbor->x();
            int ny = neighbor->y();

            if (new_g < g_costs[nx][ny])
            {
                g_costs[nx][ny] = new_g;
                neighbor->parent = current.cell;
                open.push({neighbor, new_g, new_g + heuristic(neighbor, end)});
            }
        }
    }

    LOG_ERROR("AStar: No path found!");
    return {};
}

float AStar::heuristic(Cell* from, Cell* to)
{
    float dx = std::fabs(from->x() - to->x());
    float dy = std::fabs(from->y() - to->y());
    // Octile distance
    return (dx + dy) + (std::sqrt(2.0f) - 2.0f) * std::fmin(dx, dy);
}

std::vector<Cell*> AStar::reconstructPath(Cell* start, Cell* end)
{
    std::vector<Cell*> path;
    Cell* node = end;

    while (node && node != start)
    {
        path.push_back(node);
        node = node->parent;
    }

    std::reverse(path.begin(), path.end());
    mouse_->resetPathfinding();
    return path;
}

std::string AStar::pathToString(const std::vector<Cell*>& path)
{
    std::string s;
    for (size_t i = 0; i < path.size(); i++)
    {
        s += "(" + std::to_string(path[i]->x()) + "," +
             std::to_string(path[i]->y()) + ")";
        if (i < path.size() - 1)
            s += " -> ";
    }
    return s;
}
