#ifndef APP_COMMANDS_H
#define APP_COMMANDS_H

#include <cstdint>

#include "app/motion_state.h"

/**
 * @brief Motion command types for Core0 -> Core1 communication.
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
    ARC_TURN_LEFT_90,
    ARC_TURN_RIGHT_90,
    ARC_TURN_LEFT_45,
    ARC_TURN_RIGHT_45,
    TURN_SMOOTH,
    MOVE_AHEAD,
    TURN_BACK
};

struct CommandPacket
{
    uint16_t    id;
    CommandType type;
    int32_t     param;
};

/**
 * @brief Shared-memory SPSC motion queue.
 *
 * Core0 is the only producer and Core1 is the only consumer. This mirrors the
 * MotorLab style better than using the RP2040 FIFO as a motion queue: Core1's
 * 500 Hz tick owns command acceptance and completion, and STOP is an
 * out-of-band flag checked before any queued work.
 */
class CommandHub
{
  public:
    static inline uint16_t send(CommandType type, int32_t param = 0)
    {
        if (type == CommandType::STOP)
        {
            requestStop();
            return MotionState::current_command_id;
        }

        CommandPacket packet{};
        packet.id    = nextCommandId();
        packet.type  = type;
        packet.param = param;

        while (!tryEnqueue(packet))
            tightLoopContents();

        return packet.id;
    }

    static inline void requestStop() { stop_requested_ = true; }

    static inline bool stopRequested() { return stop_requested_; }

    static inline void clearStopRequested() { stop_requested_ = false; }

    static inline bool receiveNonBlocking(CommandPacket& out)
    {
        if (tail_ == head_)
            return false;

        out   = queue_[tail_];
        tail_ = nextIndex(tail_);
        publishQueueDepth();
        return true;
    }

    static inline bool hasPending() { return head_ != tail_; }

    static inline bool canSend() { return nextIndex(head_) != tail_; }

    static inline void clear()
    {
        head_ = 0;
        tail_ = 0;
        publishQueueDepth();
    }

  private:
    static constexpr uint8_t QUEUE_SIZE = 8;

    static inline CommandPacket     queue_[QUEUE_SIZE] = {};
    static inline volatile uint8_t  head_              = 0;
    static inline volatile uint8_t  tail_              = 0;
    static inline volatile uint16_t next_id_           = 0;
    static inline volatile bool     stop_requested_    = false;

    static inline uint8_t nextIndex(uint8_t index)
    {
        return static_cast<uint8_t>((index + 1U) % QUEUE_SIZE);
    }

    static inline uint16_t nextCommandId()
    {
        ++next_id_;
        if (next_id_ == 0)
            ++next_id_;
        return next_id_;
    }

    static inline bool tryEnqueue(const CommandPacket& packet)
    {
        uint8_t next_head = nextIndex(head_);
        if (next_head == tail_)
            return false;

        queue_[head_] = packet;
        head_         = next_head;
        publishQueueDepth();
        return true;
    }

    static inline void publishQueueDepth()
    {
        uint8_t depth = (head_ >= tail_) ? (head_ - tail_) : (QUEUE_SIZE - tail_ + head_);
        MotionState::queue_depth = depth;
    }

    static inline void tightLoopContents() { __asm volatile("nop"); }
};

#endif
