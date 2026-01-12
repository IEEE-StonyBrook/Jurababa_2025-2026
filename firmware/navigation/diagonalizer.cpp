#include "navigation/diagonalizer.h"

#include <sstream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> splitPath(const std::string& path)
{
    std::vector<std::string> result;
    std::stringstream ss(path);
    std::string token;
    while (std::getline(ss, token, '#'))
    {
        if (!token.empty())
            result.push_back(token);
    }
    return result;
}

MovementBlock parseBlock(const std::string& seq)
{
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

void appendSequence(const std::string& seq, std::ostringstream& out)
{
    out << seq;
    if (!seq.empty() && seq.back() != '#')
        out << "#";
}

void executeMove(const std::vector<std::string>& moves, int& i,
                 std::ostringstream& out, MovementBlock& prev_block,
                 bool ignore_rflf = false)
{
    MovementBlock three_block = MovementBlock::DEFAULT;
    MovementBlock two_block = MovementBlock::DEFAULT;
    MovementBlock next_block = MovementBlock::DEFAULT;

    if (i + 2 < static_cast<int>(moves.size()))
    {
        three_block = parseBlock(moves[i] + moves[i + 1] + moves[i + 2]);
    }
    if (i + 1 < static_cast<int>(moves.size()))
    {
        two_block = parseBlock(moves[i] + moves[i + 1]);
    }
    next_block = parseBlock(moves[i]);

    // Handle diagonal exit transitions
    if (!ignore_rflf)
    {
        if (prev_block == MovementBlock::RFLF || prev_block == MovementBlock::LFLF)
        {
            if (two_block == MovementBlock::RF)
            {
                appendSequence("FH#R45#FH#GMF#", out);
                prev_block = two_block;
                i++;
                return;
            }
            else if (two_block == MovementBlock::LF)
            {
                appendSequence("L#FH#L45#FH#GMF#", out);
                prev_block = two_block;
                i++;
                return;
            }
            else if (three_block == MovementBlock::FLF)
            {
                appendSequence("L45#F#L45#FH#L45#FH#GFM#", out);
                prev_block = MovementBlock::F;
                i += 2;
                return;
            }
            else
            {
                appendSequence("L45#FH#", out);
                executeMove(moves, i, out, prev_block, true);
                return;
            }
        }
        else if (prev_block == MovementBlock::LFRF || prev_block == MovementBlock::RFRF)
        {
            if (two_block == MovementBlock::LF)
            {
                appendSequence("FH#L45#FH#GMF#", out);
                prev_block = two_block;
                i++;
                return;
            }
            else if (two_block == MovementBlock::RF)
            {
                appendSequence("R#FH#R45#FH#GMF#", out);
                prev_block = two_block;
                i++;
                return;
            }
            else if (three_block == MovementBlock::FRF)
            {
                appendSequence("R45#F#R45#FH#R45#FH#GMF#", out);
                prev_block = MovementBlock::F;
                i += 2;
                return;
            }
            else
            {
                appendSequence("R45#FH#", out);
                executeMove(moves, i, out, prev_block, true);
                return;
            }
        }
    }

    // Regular movement
    if (next_block == MovementBlock::R)
        appendSequence("R#", out);
    else if (next_block == MovementBlock::L)
        appendSequence("L#", out);
    else if (next_block == MovementBlock::F)
        appendSequence("F#", out);

    prev_block = next_block;
}

}  // anonymous namespace

std::string Diagonalizer::diagonalize(const std::string& path)
{
    std::ostringstream out;
    std::vector<std::string> moves = splitPath(path);
    MovementBlock block_type;
    MovementBlock prev_block = MovementBlock::DEFAULT;
    int i = 0;

    while (i < static_cast<int>(moves.size()))
    {
        // Check for forward + 4-move combo
        if (moves[i] == "F" && (i + 4 < static_cast<int>(moves.size())))
        {
            i++;
            MovementBlock next = parseBlock(moves[i] + moves[i + 1] +
                                            moves[i + 2] + moves[i + 3]);

            if (next == MovementBlock::RFRF || next == MovementBlock::LFLF ||
                next == MovementBlock::RFLF || next == MovementBlock::LFRF)
            {
                if (prev_block != MovementBlock::RFRF &&
                    prev_block != MovementBlock::LFLF &&
                    prev_block != MovementBlock::RFLF &&
                    prev_block != MovementBlock::LFRF)
                {
                    appendSequence("FH#GMF#", out);
                }
                else
                {
                    if (prev_block == MovementBlock::RFLF ||
                        prev_block == MovementBlock::LFLF)
                        appendSequence("L45#F#", out);
                    else if (prev_block == MovementBlock::LFRF ||
                             prev_block == MovementBlock::RFRF)
                        appendSequence("R45#F#", out);
                }
                prev_block = MovementBlock::F;
            }
            else
            {
                i--;  // Not a forward combo
            }
        }

        // Try 4-move combo block
        if (i + 3 < static_cast<int>(moves.size()))
        {
            block_type = parseBlock(moves[i] + moves[i + 1] +
                                    moves[i + 2] + moves[i + 3]);

            switch (block_type)
            {
                case MovementBlock::RFLF:
                    if (prev_block == block_type || prev_block == MovementBlock::LFLF)
                        appendSequence("F#", out);
                    else if (prev_block == MovementBlock::LFRF ||
                             prev_block == MovementBlock::RFRF)
                        appendSequence("R#F#", out);
                    else if (prev_block == MovementBlock::F)
                        appendSequence("R45#F#", out);
                    else
                        appendSequence("R#FH#L45#FH#GMF#", out);
                    i += 3;
                    break;

                case MovementBlock::LFRF:
                    if (prev_block == block_type || prev_block == MovementBlock::RFRF)
                        appendSequence("F#", out);
                    else if (prev_block == MovementBlock::RFLF ||
                             prev_block == MovementBlock::LFLF)
                        appendSequence("L#F#", out);
                    else if (prev_block == MovementBlock::F)
                        appendSequence("L45#F#", out);
                    else
                        appendSequence("L#FH#R45#FH#GMF#", out);
                    i += 3;
                    break;

                case MovementBlock::RFRF:
                    if (prev_block == block_type)
                        appendSequence("R#FH#R#FH#GMF#", out);
                    else if (prev_block == MovementBlock::RFLF ||
                             prev_block == MovementBlock::LFLF)
                        appendSequence("FH#R#FH#GMF#", out);
                    else if (prev_block == MovementBlock::F)
                        appendSequence("R45#FH#R#FH#GMF#", out);
                    else
                        appendSequence("R#FH#R45#FH#GMF#", out);
                    i += 3;
                    break;

                case MovementBlock::LFLF:
                    if (prev_block == block_type)
                        appendSequence("L#FH#L#FH#GMF#", out);
                    else if (prev_block == MovementBlock::LFRF ||
                             prev_block == MovementBlock::RFRF)
                        appendSequence("FH#L#FH#GMF#", out);
                    else if (prev_block == MovementBlock::F)
                        appendSequence("L45#FH#L#FH#GMF#", out);
                    else
                        appendSequence("L#FH#L45#FH#GMF#", out);
                    i += 3;
                    break;

                default:
                    executeMove(moves, i, out, prev_block);
                    break;
            }
            prev_block = block_type;
        }
        else
        {
            executeMove(moves, i, out, prev_block);
        }

        i++;
    }

    correctForEdge(out, prev_block);

    std::string result = out.str();
    if (!result.empty() && result.back() == '#')
        result.pop_back();
    return result;
}

void Diagonalizer::correctForEdge(std::ostringstream& out, MovementBlock prev_block)
{
    if (prev_block == MovementBlock::RFLF || prev_block == MovementBlock::LFLF)
    {
        out << "L45#FH#";
    }
    else if (prev_block == MovementBlock::LFRF || prev_block == MovementBlock::RFRF)
    {
        out << "R45#FH#";
    }
}
