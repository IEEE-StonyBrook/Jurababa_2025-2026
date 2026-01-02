#include "Navigation/Diagonalizer.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ------------------------------------------------------------
// Utility: split a path string ("F#R#F") into tokens
// ------------------------------------------------------------
static std::vector<std::string> splitPath(const std::string& path)
{
    std::vector<std::string> result;
    std::stringstream        ss(path);
    std::string              token;
    while (std::getline(ss, token, '#'))
    {
        if (!token.empty())
            result.push_back(token);
    }
    return result;
}

// ------------------------------------------------------------
// Utility: parse token sequence into a MovementBlock
// ------------------------------------------------------------
static MovementBlock parseMovementBlock(const std::string& seq)
{
    if (seq == "F")
        return MovementBlock::F;
    if (seq == "R")
        return MovementBlock::R;
    if (seq == "L")
        return MovementBlock::L;
    if (seq == "RF")
        return MovementBlock::RF;
    if (seq == "LF")
        return MovementBlock::LF;
    if (seq == "FRF")
        return MovementBlock::FRF;
    if (seq == "FLF")
        return MovementBlock::FLF;
    if (seq == "RFRF")
        return MovementBlock::RFRF;
    if (seq == "LFLF")
        return MovementBlock::LFLF;
    if (seq == "RFLF")
        return MovementBlock::RFLF;
    if (seq == "LFRF")
        return MovementBlock::LFRF;
    return MovementBlock::DEFAULT;
}

// ------------------------------------------------------------
// Helper: append sequence to path
// ------------------------------------------------------------
static void performSequence(const std::string& seq, std::ostringstream& diagPath)
{
    diagPath << seq;
    if (!seq.empty() && seq.back() != '#')
        diagPath << "#";
}

// ------------------------------------------------------------
// Handle individual movement with diagonal transition logic
// ------------------------------------------------------------
static void executeIndividualMovement(const std::vector<std::string>& movementsSequence, int& i,
                                      std::ostringstream& diagPath, MovementBlock& prevBlockType,
                                      bool ignoreRFLF = false)
{
    MovementBlock threeMovementBlock = MovementBlock::DEFAULT;
    MovementBlock twoMovementBlock   = MovementBlock::DEFAULT;
    MovementBlock nextMovementBlock  = MovementBlock::DEFAULT;

    if (i + 2 < static_cast<int>(movementsSequence.size()))
    {
        threeMovementBlock = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] +
                                                movementsSequence[i + 2]);
    }
    if (i + 1 < static_cast<int>(movementsSequence.size()))
    {
        twoMovementBlock = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1]);
    }
    nextMovementBlock = parseMovementBlock(movementsSequence[i]);

    // Handle transition from diagonal patterns
    if (!ignoreRFLF)
    {
        if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
        {
            if (twoMovementBlock == MovementBlock::RF)
            {
                performSequence("FH#R45#FH#GMF#", diagPath);
                prevBlockType = twoMovementBlock;
                i++;
                return;
            }
            else if (twoMovementBlock == MovementBlock::LF)
            {
                performSequence("L#FH#L45#FH#GMF#", diagPath);
                prevBlockType = twoMovementBlock;
                i++;
                return;
            }
            else if (threeMovementBlock == MovementBlock::FLF)
            {
                performSequence("L45#F#L45#FH#L45#FH#GFM#", diagPath);
                prevBlockType = MovementBlock::F;
                i += 2;
                return;
            }
            else
            {
                // Exit diagonal with edge correction, then handle movement
                performSequence("L45#FH#", diagPath);
                executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, true);
                return;
            }
        }
        else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
        {
            if (twoMovementBlock == MovementBlock::LF)
            {
                performSequence("FH#L45#FH#GMF#", diagPath);
                prevBlockType = twoMovementBlock;
                i++;
                return;
            }
            else if (twoMovementBlock == MovementBlock::RF)
            {
                performSequence("R#FH#R45#FH#GMF#", diagPath);
                prevBlockType = twoMovementBlock;
                i++;
                return;
            }
            else if (threeMovementBlock == MovementBlock::FRF)
            {
                performSequence("R45#F#R45#FH#R45#FH#GMF#", diagPath);
                prevBlockType = MovementBlock::F;
                i += 2;
                return;
            }
            else
            {
                // Exit diagonal with edge correction, then handle movement
                performSequence("R45#FH#", diagPath);
                executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, true);
                return;
            }
        }
    }

    // Regular movement handling
    if (nextMovementBlock == MovementBlock::R)
    {
        performSequence("R#", diagPath);
    }
    else if (nextMovementBlock == MovementBlock::L)
    {
        performSequence("L#", diagPath);
    }
    else if (nextMovementBlock == MovementBlock::F)
    {
        performSequence("F#", diagPath);
    }
    prevBlockType = nextMovementBlock;
}

// ------------------------------------------------------------
// Main diagonalizer: returns a diagonalized path string
// ------------------------------------------------------------
std::string Diagonalizer::diagonalize(const std::string& path)
{
    std::ostringstream       diagPath;
    std::vector<std::string> movementsSequence = splitPath(path);
    MovementBlock            blockType;
    MovementBlock            prevBlockType = MovementBlock::DEFAULT;
    int                      i             = 0;

    while (i < static_cast<int>(movementsSequence.size()))
    {
        // Check for a forward combo: "F" followed by a 4-move combo block.
        if (movementsSequence[i] == "F" && (i + 4 < static_cast<int>(movementsSequence.size())))
        {
            i++;
            MovementBlock nextMovementBlock =
                parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] +
                                   movementsSequence[i + 2] + movementsSequence[i + 3]);
            if (nextMovementBlock == MovementBlock::RFRF ||
                nextMovementBlock == MovementBlock::LFLF ||
                nextMovementBlock == MovementBlock::RFLF ||
                nextMovementBlock == MovementBlock::LFRF)
            {
                if (prevBlockType != MovementBlock::RFRF && prevBlockType != MovementBlock::LFLF &&
                    prevBlockType != MovementBlock::RFLF && prevBlockType != MovementBlock::LFRF)
                {
                    performSequence("FH#GMF#", diagPath);
                }
                else
                {
                    if (prevBlockType == MovementBlock::RFLF ||
                        prevBlockType == MovementBlock::LFLF)
                    {
                        performSequence("L45#F#", diagPath);
                    }
                    else if (prevBlockType == MovementBlock::LFRF ||
                             prevBlockType == MovementBlock::RFRF)
                    {
                        performSequence("R45#F#", diagPath);
                    }
                }
                prevBlockType = MovementBlock::F;
            }
            else
            {
                i--; // not a forward combo, back up
            }
        }

        // Attempt to form a 4-move combo block if possible.
        if (i + 3 < static_cast<int>(movementsSequence.size()))
        {
            blockType = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] +
                                           movementsSequence[i + 2] + movementsSequence[i + 3]);

            switch (blockType)
            {
                case MovementBlock::RFLF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::LFLF)
                        performSequence("F#", diagPath);
                    else if (prevBlockType == MovementBlock::LFRF ||
                             prevBlockType == MovementBlock::RFRF)
                        performSequence("R#F#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("R45#F#", diagPath);
                    else
                        performSequence("R#FH#L45#FH#GMF#", diagPath);
                    i += 3;
                    break;

                case MovementBlock::LFRF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::RFRF)
                        performSequence("F#", diagPath);
                    else if (prevBlockType == MovementBlock::RFLF ||
                             prevBlockType == MovementBlock::LFLF)
                        performSequence("L#F#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("L45#F#", diagPath);
                    else
                        performSequence("L#FH#R45#FH#GMF#", diagPath);
                    i += 3;
                    break;

                case MovementBlock::RFRF:
                    if (prevBlockType == blockType)
                        performSequence("R#FH#R#FH#GMF#", diagPath);
                    else if (prevBlockType == MovementBlock::RFLF ||
                             prevBlockType == MovementBlock::LFLF)
                        performSequence("FH#R#FH#GMF#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("R45#FH#R#FH#GMF#", diagPath);
                    else
                        performSequence("R#FH#R45#FH#GMF#", diagPath);
                    i += 3;
                    break;

                case MovementBlock::LFLF:
                    if (prevBlockType == blockType)
                        performSequence("L#FH#L#FH#GMF#", diagPath);
                    else if (prevBlockType == MovementBlock::LFRF ||
                             prevBlockType == MovementBlock::RFRF)
                        performSequence("FH#L#FH#GMF#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("L45#FH#L#FH#GMF#", diagPath);
                    else
                        performSequence("L#FH#L45#FH#GMF#", diagPath);
                    i += 3;
                    break;

                default:
                    // No 4-move diagonal pattern - handle with transition logic
                    executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType);
                    break;
            }
            prevBlockType = blockType;
        }
        else
        {
            // Not enough tokens for 4-move pattern - handle with transition logic
            executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType);
        }

        i++;
    }

    correctForEdge(diagPath, prevBlockType);

    std::string out = diagPath.str();
    if (!out.empty() && out.back() == '#')
        out.pop_back();
    return out;
}

// ------------------------------------------------------------
// Edge correction implementation
// ------------------------------------------------------------
void Diagonalizer::correctForEdge(std::ostringstream& diagPath, MovementBlock prevBlockType)
{
    if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
    {
        diagPath << "L45#FH#";
    }
    else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
    {
        diagPath << "R45#FH#";
    }
}
