#pragma once
#include <sstream>
#include <string>

enum class MovementBlock {
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

class Diagonalizer {
public:
    static std::string diagonalize(const std::string& path);
    static void correctForEdge(std::ostringstream& diagPath, MovementBlock prevBlockType);
};