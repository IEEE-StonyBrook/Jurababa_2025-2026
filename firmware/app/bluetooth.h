#ifndef APP_BLUETOOTH_H
#define APP_BLUETOOTH_H

#include <cstdint>
#include <string>

#include "hardware/uart.h"

/**
 * @brief Non-blocking Bluetooth serial interface using UART with interrupt-driven RX
 *
 * Provides wireless serial communication for logging and remote commands.
 * Uses interrupt-driven receive to capture commands without blocking.
 *
 * Hardware config:
 *   - UART0 at 9600 baud (default)
 *   - TX/RX pins configurable
 *
 * Supported commands (single character):
 *   - 's': Start command
 *   - 'h': Halt command
 *   - 'r': Reset command
 *   - 'b': Battery status request
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

    Bluetooth(uart_inst_t* uart = uart0, uint32_t baud_rate = 9600, uint8_t tx_pin = 0,
              uint8_t rx_pin = 1);

    void init();

    void write(const std::string& data);
    void write(const char* data);
    void writeBytes(const uint8_t* data, size_t length);

    bool    hasCommand() const;
    Command command();
    char    lastChar() const;
    bool    txReady() const;

    static Bluetooth* instance_;

  private:
    static void rxInterruptHandler();
    void        processChar(char c);

    uart_inst_t* uart_;
    uint32_t     baud_rate_;
    uint8_t      tx_pin_;
    uint8_t      rx_pin_;

    volatile Command pending_command_;
    volatile char    last_char_;
    volatile bool    command_ready_;
};

#endif
