#ifndef DIAGONALIZER_H
#define DIAGONALIZER_H

#include <string>

class Diagonalizer {
public:
    // Converts an LFR-style path string (e.g., "F#R45#F#L45#F#")
    // into a diagonalized path string (e.g., "FH#FH#R#F#")
    std::string diagonalize(const std::string& path);
};

#endif
