#pragma once

#include "pico/multicore.h"
#include <cstdint>

// Define the command types (auto-increment is fine)
enum class CommandType : uint8_t {
    NONE,       // 0
    MOVE_FWD,   // 1
    TURN_LEFT,  // 2
    TURN_RIGHT, // 3
    STOP,       // 4
    TURN_ARBITRARY, // 5
    MOVE_FWD_HALF // 6
};

// Command packet structure
struct CommandPacket {
    CommandType type;
    int32_t param; // optional parameter (speed, distance, angle, etc.)
};

// Hub for sending/receiving commands between cores
class CommandHub {
public:
    // Send a command from Core0 → Core1
    static inline void send(CommandType type, int32_t param = 0) {
        uint32_t packed = (static_cast<uint32_t>(type) << 24) |
                          (static_cast<uint32_t>(param) & 0xFFFFFF);
        multicore_fifo_push_blocking(packed);
    }

    // Blocking receive (Core1 waits for a command)
    static inline CommandPacket receiveBlocking() {
        uint32_t packed = multicore_fifo_pop_blocking();
        return unpack(packed);
    }

    // Non-blocking receive (returns true if a command was available)
    static inline bool receiveNonBlocking(CommandPacket &out) {
        if (multicore_fifo_rvalid()) {
            uint32_t packed = multicore_fifo_pop_blocking();
            out = unpack(packed);
            return true;
        }
        return false;
    }

    static inline bool hasPendingCommands() {
        return multicore_fifo_rvalid();
    }

    static inline bool hasSpaceToSend() {
        return multicore_fifo_wready();
    }

private:
    // Helper: unpack 32-bit word into command
    static inline CommandPacket unpack(uint32_t packed) {
        CommandPacket cmd;
        cmd.type = static_cast<CommandType>((packed >> 24) & 0xFF);
        int32_t raw = static_cast<int32_t>(packed & 0xFFFFFF);
        // sign-extend 24-bit → 32-bit
        if (raw & 0x800000) raw |= 0xFF000000;
        cmd.param = raw;
        return cmd;
    }
};
