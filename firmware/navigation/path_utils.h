#ifndef NAVIGATION_PATH_UTILS_H
#define NAVIGATION_PATH_UTILS_H

#include <array>
#include <string>
#include <vector>

class Mouse;
class MazeMouse;
class Cell;

/**
 * @brief Path execution and utility functions for maze navigation
 */
namespace PathUtils
{

/**
 * @brief Traverse path to goals with iterative re-planning
 * @param mouse Mouse interface for movement
 * @param maze_mouse MazeMouse state tracker
 * @param goals Target cell coordinates
 * @param diagonals Allow diagonal movement
 * @param all_explored Treat maze as fully explored
 * @param avoid_goals Don't pass through goal cells
 * @return True if goal reached successfully
 */
bool traversePath(Mouse* mouse, MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals,
                  bool diagonals, bool all_explored, bool avoid_goals);

// Execute one cardinal speed path using only cells already explored.
bool traverseExploredPath(Mouse* mouse, MazeMouse* maze_mouse,
                          const std::vector<std::array<int, 2>>& goals);

// Execute one diagonalized speed path using only cells already explored.
bool traverseExploredDiagonalPath(Mouse* mouse, MazeMouse* maze_mouse,
                                  const std::vector<std::array<int, 2>>& goals);

/**
 * @brief Detect walls around current position and update maze
 * @param mouse Mouse interface for wall sensing
 * @param maze_mouse MazeMouse state tracker
 */
void detectWalls(Mouse& mouse, MazeMouse& maze_mouse);

/**
 * @brief Color path cells on display
 * @param mouse Mouse interface for display
 * @param path Cells to color
 */
void colorPath(Mouse* mouse, const std::vector<Cell*>& path);

} // namespace PathUtils

#endif
