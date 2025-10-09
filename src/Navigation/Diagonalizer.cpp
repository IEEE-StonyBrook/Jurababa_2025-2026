#include "../../Include/Navigation/Diagonalizer.h"

#include <queue>
#include <sstream>
#include <string>

// Simple state machine that walks through the path tokens
std::string Diagonalizer::diagonalize(const std::string& path) {
    std::stringstream input(path);
    std::string token;
    std::string prev;
    std::string output;
    enum State { NORMAL, DIAGONAL } state = NORMAL;

    while (std::getline(input, token, '#')) {
        if (token.empty()) continue;

        // --- Detect transitions into diagonals ---
        if (state == NORMAL) {
            if (token == "F") {
                prev = token;  // might be start of diagonal
                continue;
            } else if (token == "R45" || token == "L45") {
                // Found diagonal start after forward
                if (prev == "F") {
                    output += "FH#";
                    state = DIAGONAL;
                    prev.clear();
                    continue;
                }
            }
        }

        // --- Inside diagonal sequence ---
        if (state == DIAGONAL) {
            // If diagonal ends (encounter a normal forward, turn, etc.)
            if (token == "R" || token == "L" || token == "B") {
                state = NORMAL;
                output += token + "#";
            } else if (token == "F") {
                // Extend diagonal straight
                output += "FH#";
            } else {
                // Stay diagonal for 45 turns
                output += token + "#";
            }
            continue;
        }

        // --- Normal sequence (no diagonal) ---
        if (!prev.empty()) {
            output += prev + "#";
            prev.clear();
        }
        output += token + "#";
    }

    // Add any trailing token
    if (!prev.empty()) output += prev + "#";

    // Remove last '#'
    if(!output.empty() && output.back() == '#') output.pop_back();

    return output;
}
