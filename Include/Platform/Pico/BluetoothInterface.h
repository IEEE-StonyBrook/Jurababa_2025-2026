#ifndef BLUETOOTHINTERFACE_H
#define BLUETOOTHINTERFACE_H

#include <cstdint>
#include <string>

#include "hardware/uart.h"

/**
 * @class BluetoothInterface
 * @brief Non-blocking Bluetooth serial interface using UART0 with interrupt-driven RX.
 *
 * Provides wireless serial communication for logging and remote commands.
 * Uses interrupt-driven receive to capture commands without blocking.
 *
 * Hardware config:
 *   - UART0 at 9600 baud
 *   - TX: GP0
 *   - RX: GP1
 *
 * Supported commands (single character):
 *   - 's': Start command
 *   - 'h': Halt command
 *   - 'r': Reset command
 *   - 'b': Battery status request
 */
class BluetoothInterface {
  public:
    /**
     * @brief Command types received via Bluetooth.
     */
    enum class Command : uint8_t {
        NONE = 0,    ///< No command pending
        START,       ///< Start/resume operation
        HALT,        ///< Stop/pause operation
        RESET,       ///< Reset system
        BATTERY,     ///< Request battery status
        UNKNOWN      ///< Unrecognized command
    };

    /**
     * @brief Constructs a BluetoothInterface.
     *
     * @param uart_instance UART instance (uart0 or uart1).
     * @param baud_rate Baud rate (default 9600).
     * @param tx_pin GPIO pin for TX (default GP0).
     * @param rx_pin GPIO pin for RX (default GP1).
     */
    BluetoothInterface(uart_inst_t* uart_instance = uart0,
                       uint32_t baud_rate = 9600,
                       uint8_t tx_pin = 0,
                       uint8_t rx_pin = 1);

    /**
     * @brief Initializes UART hardware and enables RX interrupt.
     */
    void init();

    /**
     * @brief Sends a string over Bluetooth (non-blocking for small strings).
     *
     * @param data String to transmit.
     */
    void write(const std::string& data);

    /**
     * @brief Sends a C-string over Bluetooth.
     *
     * @param data Null-terminated C-string to transmit.
     */
    void write(const char* data);

    /**
     * @brief Sends raw bytes over Bluetooth.
     *
     * @param data Pointer to byte buffer.
     * @param length Number of bytes to send.
     */
    void writeBytes(const uint8_t* data, size_t length);

    /**
     * @brief Checks if a command has been received (non-blocking).
     *
     * @return true if a command is pending, false otherwise.
     */
    bool hasCommand() const;

    /**
     * @brief Gets and clears the pending command.
     *
     * @return Command The received command (NONE if no command pending).
     */
    Command getCommand();

    /**
     * @brief Gets the last received raw character.
     *
     * @return char The last character received via interrupt.
     */
    char getLastChar() const;

    /**
     * @brief Checks if UART TX FIFO is ready for more data.
     *
     * @return true if TX is ready (non-blocking write possible).
     */
    bool isTxReady() const;

    /**
     * @brief Static instance pointer for interrupt handler access.
     */
    static BluetoothInterface* instance_;

  private:
    /**
     * @brief UART RX interrupt handler (static for C callback).
     */
    static void rxInterruptHandler();

    /**
     * @brief Processes a received character into a command.
     *
     * @param c The received character.
     */
    void processReceivedChar(char c);

    uart_inst_t* uart_;              ///< UART instance.
    uint32_t     baud_rate_;         ///< Configured baud rate.
    uint8_t      tx_pin_;            ///< TX GPIO pin.
    uint8_t      rx_pin_;            ///< RX GPIO pin.

    volatile Command pending_command_;  ///< Command received via interrupt.
    volatile char    last_char_;        ///< Last character received.
    volatile bool    command_ready_;    ///< Flag indicating command is pending.
};

#endif  // BLUETOOTHINTERFACE_H
