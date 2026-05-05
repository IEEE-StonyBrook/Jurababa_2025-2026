#ifndef APP_BLUETOOTH_H
#define APP_BLUETOOTH_H

#include <cstdint>
#include <string>

#include "hardware/uart.h"
#include "pico/critical_section.h"

/**
 * @brief Non-blocking Bluetooth serial interface using UART with interrupt-driven RX
 *
 * Provides wireless serial communication for logging and remote commands.
 * Uses interrupt-driven receive to capture commands without blocking.
 *
 * Hardware config:
 *   - UART0 at 115200 baud (default; HC-05 must be reconfigured via
 *     `AT+UART=115200,0,0` in AT mode — see runDriverLabMode comment)
 *   - TX/RX pins configurable
 *
 * Supported immediate commands (single character, no newline):
 *   - 's': Start command
 *   - 'h': Halt command
 *   - 'r': Reset command
 *   - 'b': Battery status request
 *
 * Newline-terminated text is queued for the Normal CLI parser. This keeps
 * terminal-style Bluetooth input on the same command path as USB serial.
 */
class Bluetooth
{
  public:
    enum class Command : uint8_t
    {
        NONE = 0,
        START,
        HALT,
        RESET,
        BATTERY,
        UNKNOWN
    };

    Bluetooth(uart_inst_t* uart = uart0, uint32_t baud_rate = 115200, uint8_t tx_pin = 0,
              uint8_t rx_pin = 1);

    void init();

    void write(const std::string& data);
    void write(const char* data);
    void writeBytes(const uint8_t* data, size_t length);
    void drain();

    struct Diagnostics
    {
        uint16_t ring_depth;
        uint16_t max_ring_depth;
        uint32_t dropped_bytes;
        uint16_t rx_line_depth;
        uint16_t max_rx_line_depth;
        uint32_t dropped_rx_lines;
        uint32_t dropped_rx_chars;
        bool     pending_shortcut;
    };

    Diagnostics diagnostics() const;

    bool    hasCommand();
    Command command();
    bool    hasLine() const;
    bool    readLine(char* out, size_t out_length);

    static Bluetooth* instance_;

  private:
    static void rxInterruptHandler();
    void        processChar(char c);
    void        enqueueByte(uint8_t byte);
    uint16_t    ringDepthLocked() const;
    void        appendRxChar(char c);
    void        enqueueRxLine();
    bool        setPendingCommand(Command command);
    bool        promotePendingShortcut();
    uint16_t    rxLineDepth() const;

    uart_inst_t* uart_;
    uint32_t     baud_rate_;
    uint8_t      tx_pin_;
    uint8_t      rx_pin_;

    volatile Command pending_command_;
    volatile bool    command_ready_;

    static constexpr uint16_t TX_RING_SIZE        = 4096;
    static constexpr uint16_t RX_LINE_BUFFER_SIZE = 96;
    static constexpr uint16_t RX_LINE_QUEUE_SIZE  = 4;
    static constexpr uint32_t SHORTCUT_DELAY_US   = 500000;

    struct RxLine
    {
        char     text[RX_LINE_BUFFER_SIZE];
        uint16_t length;
    };

    static uint8_t             tx_ring_[TX_RING_SIZE];
    mutable critical_section_t tx_lock_;
    volatile uint16_t          tx_head_          = 0;
    volatile uint16_t          tx_tail_          = 0;
    volatile uint16_t          max_tx_depth_     = 0;
    volatile uint32_t          dropped_tx_bytes_ = 0;

    RxLine            rx_lines_[RX_LINE_QUEUE_SIZE]        = {};
    char              rx_line_buffer_[RX_LINE_BUFFER_SIZE] = {};
    volatile uint16_t rx_line_length_                      = 0;
    volatile uint16_t rx_line_head_                        = 0;
    volatile uint16_t rx_line_tail_                        = 0;
    volatile uint16_t max_rx_line_depth_                   = 0;
    volatile uint32_t dropped_rx_lines_                    = 0;
    volatile uint32_t dropped_rx_chars_                    = 0;

    volatile bool     pending_shortcut_      = false;
    volatile char     pending_shortcut_char_ = '\0';
    volatile uint32_t pending_shortcut_us_   = 0;
};

#endif
