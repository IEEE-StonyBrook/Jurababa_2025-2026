#ifndef NAVIGATION_PATH_UTILS_H
#define NAVIGATION_PATH_UTILS_H

#include <array>
#include <string>
#include <vector>

class API;
class Mouse;
class Cell;

/**
 * @brief Path execution and utility functions for maze navigation
 */
namespace PathUtils
{

/**
 * @brief Execute an LFR path string
 * @param api API interface for movement commands
 * @param lfr_path Path string (e.g., "F#R#F#L#F")
 */
void executePath(API* api, const std::string& lfr_path);

/**
 * @brief Traverse path to goals with iterative re-planning
 * @param api API interface for movement
 * @param mouse Mouse state tracker
 * @param goals Target cell coordinates
 * @param diagonals Allow diagonal movement
 * @param all_explored Treat maze as fully explored
 * @param avoid_goals Don't pass through goal cells
 * @return True if goal reached successfully
 */
bool traversePath(API* api, Mouse* mouse,
                  const std::vector<std::array<int, 2>>& goals,
                  bool diagonals, bool all_explored, bool avoid_goals);

/**
 * @brief Mark all cells in maze as explored
 * @param mouse Mouse with maze reference
 */
void setAllExplored(Mouse* mouse);

/**
 * @brief Detect walls around current position and update maze
 * @param api API interface for wall sensing
 * @param mouse Mouse state tracker
 */
void detectWalls(API& api, Mouse& mouse);

/**
 * @brief Color path cells on display
 * @param api API interface for display
 * @param path Cells to color
 */
void colorPath(API* api, const std::vector<Cell*>& path);

}  // namespace PathUtils

#endif
