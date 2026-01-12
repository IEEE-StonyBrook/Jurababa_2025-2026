#ifndef APP_COMMANDS_H
#define APP_COMMANDS_H

#include "pico/multicore.h"
#include <cstdint>

/**
 * @brief Command types for inter-core communication
 */
enum class CommandType : uint8_t
{
    NONE,
    MOVE_FWD,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
    TURN_ARBITRARY,
    MOVE_FWD_HALF,
    CENTER_FROM_EDGE,
    SNAPSHOT,
    ARC_TURN_LEFT_90,
    ARC_TURN_RIGHT_90,
    ARC_TURN_LEFT_45,
    ARC_TURN_RIGHT_45
};

/**
 * @brief Command packet for inter-core messages
 */
struct CommandPacket
{
    CommandType type;
    int32_t param;
};

/**
 * @brief Inter-core command hub using RP2040 multicore FIFO
 *
 * Packs command type (8 bits) and parameter (24 bits) into a single
 * 32-bit word for efficient FIFO transfer between cores.
 */
class CommandHub
{
  public:
    static inline void send(CommandType type, int32_t param = 0)
    {
        uint32_t packed =
            (static_cast<uint32_t>(type) << 24) |
            (static_cast<uint32_t>(param) & 0xFFFFFF);
        multicore_fifo_push_blocking(packed);
    }

    static inline CommandPacket receiveBlocking()
    {
        uint32_t packed = multicore_fifo_pop_blocking();
        return unpack(packed);
    }

    static inline bool receiveNonBlocking(CommandPacket& out)
    {
        if (multicore_fifo_rvalid())
        {
            uint32_t packed = multicore_fifo_pop_blocking();
            out = unpack(packed);
            return true;
        }
        return false;
    }

    static inline bool hasPending() { return multicore_fifo_rvalid(); }
    static inline bool canSend() { return multicore_fifo_wready(); }

  private:
    static inline CommandPacket unpack(uint32_t packed)
    {
        CommandPacket cmd;
        cmd.type = static_cast<CommandType>((packed >> 24) & 0xFF);
        int32_t raw = static_cast<int32_t>(packed & 0xFFFFFF);
        // Sign-extend 24-bit to 32-bit
        if (raw & 0x800000)
            raw |= 0xFF000000;
        cmd.param = raw;
        return cmd;
    }
};

#endif
