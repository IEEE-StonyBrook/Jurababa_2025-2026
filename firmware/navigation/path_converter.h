#ifndef NAVIGATION_PATH_CONVERTER_H
#define NAVIGATION_PATH_CONVERTER_H

#include <array>
#include <string>
#include <vector>

class Cell;

/**
 * @brief Converts cell paths to LFR command strings
 *
 * Takes a sequence of cells and generates Left/Forward/Right
 * movement commands based on heading changes.
 */
class PathConverter
{
  public:
    /**
     * @brief Build LFR path string from cell sequence
     * @param start_cell Starting cell position
     * @param start_heading Initial direction as [dx, dy]
     * @param path Vector of cells to traverse
     * @return LFR-formatted string (e.g., "F#R#F#L#F")
     */
    static std::string buildLFR(Cell* start_cell,
                                std::array<int, 2> start_heading,
                                const std::vector<Cell*>& path);

  private:
    static std::string movesForHeadingChange(std::array<int, 2> from,
                                             std::array<int, 2> to);
    static std::string cardinalMoves(std::array<int, 2> from,
                                     std::array<int, 2> to);
    static int findDirectionIndex(const std::vector<std::array<int, 2>>& dirs,
                                  std::array<int, 2> dir);

    static Cell* current_cell_;
    static std::array<int, 2> current_heading_;
};

#endif
