#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include "Navigation/Diagonalizer.h"

// ------------------------------------------------------------
// Utility: split a path string ("F#R#F") into tokens
// ------------------------------------------------------------
static std::vector<std::string> splitPath(const std::string& path) {
    std::vector<std::string> result;
    std::stringstream ss(path);
    std::string token;
    while (std::getline(ss, token, '#')) {
        if (!token.empty()) result.push_back(token);
    }
    return result;
}

// ------------------------------------------------------------
// Utility: parse token sequence into a MovementBlock
// ------------------------------------------------------------
static MovementBlock parseMovementBlock(const std::string& seq) {
    if (seq == "F") return MovementBlock::F;
    if (seq == "R") return MovementBlock::R;
    if (seq == "L") return MovementBlock::L;
    if (seq == "RF") return MovementBlock::RF;
    if (seq == "LF") return MovementBlock::LF;
    if (seq == "FRF") return MovementBlock::FRF;
    if (seq == "FLF") return MovementBlock::FLF;
    if (seq == "RFRF") return MovementBlock::RFRF;
    if (seq == "LFLF") return MovementBlock::LFLF;
    if (seq == "RFLF") return MovementBlock::RFLF;
    if (seq == "LFRF") return MovementBlock::LFRF;
    return MovementBlock::DEFAULT;
}

// ------------------------------------------------------------
// Debug helper
// ------------------------------------------------------------
static void debugLog(const std::string& msg) {
    std::cout << "[Diagonalizer] " << msg << std::endl;
}

// ------------------------------------------------------------
// Main diagonalizer: returns a diagonalized path string
// ------------------------------------------------------------
std::string Diagonalizer::diagonalize(const std::string& path) {
    std::ostringstream diagPath;
    std::vector<std::string> movementsSequence = splitPath(path);
    MovementBlock blockType;
    MovementBlock prevBlockType = MovementBlock::DEFAULT;
    int i = 0;

    // debugLog("Split path into tokens:");
    for (auto& t : movementsSequence) {
        // debugLog("  token = " + t);
    }

    auto append = [&](const std::string& seq, const std::string& reason) {
        diagPath << seq;
        if (!seq.empty() && seq.back() != '#') diagPath << "#";
        // debugLog("Appending \"" + seq + "\" because: " + reason);
    };

    while (i < static_cast<int>(movementsSequence.size())) {
        // --- Forward combos (look ahead 4 moves) ---
        if (movementsSequence[i] == "F" && (i + 4 < (int)movementsSequence.size())) {
            i++;
            MovementBlock nextMovementBlock =
                parseMovementBlock(movementsSequence[i] +
                                   movementsSequence[i + 1] +
                                   movementsSequence[i + 2] +
                                   movementsSequence[i + 3]);
            if (nextMovementBlock == MovementBlock::RFRF ||
                nextMovementBlock == MovementBlock::LFLF ||
                nextMovementBlock == MovementBlock::RFLF ||
                nextMovementBlock == MovementBlock::LFRF) {
                if (prevBlockType != MovementBlock::RFRF &&
                    prevBlockType != MovementBlock::LFLF &&
                    prevBlockType != MovementBlock::RFLF &&
                    prevBlockType != MovementBlock::LFRF) {
                    append("FH#", "start of a diagonal combo");
                } else {
                    if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF) {
                        append("L45#F#", "after forward block");
                    } else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF) {
                        append("R45#F#", "after forward block");
                    }
                }
                prevBlockType = MovementBlock::F;
            } else {
                i--; // not a forward combo
            }
        }

        // --- Attempt to form 4-move combo block ---
        if (i + 3 < (int)movementsSequence.size()) {
            blockType = parseMovementBlock(movementsSequence[i] +
                                           movementsSequence[i + 1] +
                                           movementsSequence[i + 2] +
                                           movementsSequence[i + 3]);

            switch (blockType) {
                case MovementBlock::RFLF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::LFLF)
                        append("F#", "continuing RFLF or LFLF");
                    else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
                        append("R#F#", "after LFRF/RFRF");
                    else if (prevBlockType == MovementBlock::F)
                        append("R45#F#", "after forward block");
                    else
                        append("R#FH#L45#FH#", "new RFLF combo");
                    i += 3;
                    break;

                case MovementBlock::LFRF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::RFRF)
                        append("F#", "continuing LFRF or RFRF");
                    else if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
                        append("L#F#", "after RFLF/LFLF");
                    else if (prevBlockType == MovementBlock::F)
                        append("L45#F#", "after forward block");
                    else
                        append("L#FH#R45#FH#", "new LFRF combo");
                    i += 3;
                    break;

                case MovementBlock::RFRF:
                    if (prevBlockType == blockType)
                        append("R#FH#R#FH#", "continuing RFRF");
                    else if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
                        append("FH#R#FH#", "after RFLF/LFLF");
                    else if (prevBlockType == MovementBlock::F)
                        append("R45#FH#R#FH#", "after forward block");
                    else
                        append("R#FH#R45#FH#", "new RFRF combo");
                    i += 3;
                    break;

                case MovementBlock::LFLF:
                    if (prevBlockType == blockType)
                        append("L#FH#L#FH#", "continuing LFLF");
                    else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
                        append("FH#L#FH#", "after LFRF/RFRF");
                    else if (prevBlockType == MovementBlock::F)
                        append("L45#FH#L#FH#", "after forward block");
                    else
                        append("L#FH#L45#FH#", "new LFLF combo");
                    i += 3;
                    break;

                default:
                    // No diagonal pattern matched - preserve the original movement
                    append(movementsSequence[i] + "#", "no diagonal pattern - keeping original");
                    break;
            }
            prevBlockType = blockType;
        } else {
            // Not enough tokens for a 4-move pattern - preserve original movement
            append(movementsSequence[i] + "#", "leftover at end - keeping original");
        }

        i++;
    }

    // debugLog("Correcting for edge...");
    correctForEdge(diagPath, prevBlockType);

    std::string out = diagPath.str();
    if (!out.empty() && out.back() == '#') out.pop_back();
    // debugLog("Final diagonalized path: " + out);
    return out;
}

// ------------------------------------------------------------
// Edge correction implementation
// ------------------------------------------------------------
void Diagonalizer::correctForEdge(std::ostringstream& diagPath, MovementBlock prevBlockType) {
    if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF) {
        // debugLog("Edge correction: ending with L45#FH#");
        diagPath << "L45#FH#";
    } else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF) {
        // debugLog("Edge correction: ending with R45#FH#");
        diagPath << "R45#FH#";
    }
}