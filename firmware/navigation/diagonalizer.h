#ifndef NAVIGATION_DIAGONALIZER_H
#define NAVIGATION_DIAGONALIZER_H

#include <sstream>
#include <string>

/**
 * @brief Movement block types for diagonal path optimization
 */
enum class MovementBlock
{
    DEFAULT,
    F,
    R,
    L,
    RF,
    LF,
    FRF,
    FLF,
    RFRF,
    LFLF,
    RFLF,
    LFRF
};

/**
 * @brief Converts cardinal LFR paths to diagonal paths
 *
 * Analyzes sequences of Left/Forward/Right moves and converts
 * them to optimized diagonal movements using 45-degree turns
 * and half-cell movements.
 */
class Diagonalizer
{
  public:
    /**
     * @brief Convert cardinal path to diagonal path
     * @param path LFR path string (e.g., "F#R#F#L#F")
     * @return Diagonalized path with 45-degree turns and half moves
     */
    static std::string diagonalize(const std::string& path);

    /**
     * @brief Append edge correction to path end
     * @param diag_path Output stream for diagonal path
     * @param prev_block Previous movement block type
     */
    static void correctForEdge(std::ostringstream& diag_path, MovementBlock prev_block);
};

#endif
