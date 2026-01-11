#include "Platform/Pico/BluetoothInterface.h"

#include <cstring>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"

// Static instance pointer for interrupt access
BluetoothInterface* BluetoothInterface::instance_ = nullptr;

BluetoothInterface::BluetoothInterface(uart_inst_t* uart_instance,
                                       uint32_t baud_rate,
                                       uint8_t tx_pin,
                                       uint8_t rx_pin)
    : uart_(uart_instance),
      baud_rate_(baud_rate),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      pending_command_(Command::NONE),
      last_char_(0),
      command_ready_(false)
{
    instance_ = this;
}

void BluetoothInterface::init()
{
    printf("[BT] Starting Bluetooth init...\n");
    printf("[BT] UART instance: %s\n", (uart_ == uart0) ? "uart0" : "uart1");
    printf("[BT] Baud rate: %lu\n", baud_rate_);
    printf("[BT] TX pin: %d, RX pin: %d\n", tx_pin_, rx_pin_);

    // Initialize UART with specified baud rate
    printf("[BT] Step 1: Calling uart_init()...\n");
    uint actual_baud = uart_init(uart_, baud_rate_);
    printf("[BT] Step 1 done. Actual baud rate: %u\n", actual_baud);

    // Configure GPIO pins for UART function
    printf("[BT] Step 2: Setting GPIO functions for TX/RX pins...\n");
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    printf("[BT] Step 2 done.\n");

    // Set UART format: 8 data bits, 1 stop bit, no parity
    printf("[BT] Step 3: Setting UART format (8N1)...\n");
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    printf("[BT] Step 3 done.\n");

    // Enable FIFO for better performance
    printf("[BT] Step 4: Enabling FIFO...\n");
    uart_set_fifo_enabled(uart_, true);
    printf("[BT] Step 4 done.\n");

    // Determine IRQ number based on UART instance
    int uart_irq = (uart_ == uart0) ? UART0_IRQ : UART1_IRQ;
    printf("[BT] Step 5: Setting up IRQ %d...\n", uart_irq);

    // Set up interrupt handler
    irq_set_exclusive_handler(uart_irq, rxInterruptHandler);
    irq_set_enabled(uart_irq, true);
    printf("[BT] Step 5 done. IRQ handler registered.\n");

    // Enable RX interrupt only (TX handled synchronously)
    printf("[BT] Step 6: Enabling RX interrupt...\n");
    uart_set_irq_enables(uart_, true, false);
    printf("[BT] Step 6 done.\n");

    printf("[BT] Bluetooth init complete!\n");

    // Test TX by sending directly
    printf("[BT] Sending test message over UART TX...\n");
    uart_puts(uart_, "BT_INIT_OK\r\n");
    printf("[BT] Test message sent.\n");
}

void BluetoothInterface::write(const std::string& data)
{
    printf("[BT] write(string): \"%s\"\n", data.c_str());
    uart_puts(uart_, data.c_str());
    printf("[BT] write(string) done.\n");
}

void BluetoothInterface::write(const char* data)
{
    printf("[BT] write(char*): \"%s\"\n", data);
    uart_puts(uart_, data);
    printf("[BT] write(char*) done.\n");
}

void BluetoothInterface::writeBytes(const uint8_t* data, size_t length)
{
    printf("[BT] writeBytes: %zu bytes\n", length);
    uart_write_blocking(uart_, data, length);
    printf("[BT] writeBytes done.\n");
}

bool BluetoothInterface::hasCommand() const
{
    return command_ready_;
}

BluetoothInterface::Command BluetoothInterface::getCommand()
{
    if (!command_ready_)
    {
        return Command::NONE;
    }

    Command cmd    = pending_command_;
    command_ready_ = false;
    pending_command_ = Command::NONE;
    return cmd;
}

char BluetoothInterface::getLastChar() const
{
    return last_char_;
}

bool BluetoothInterface::isTxReady() const
{
    return uart_is_writable(uart_);
}

void BluetoothInterface::rxInterruptHandler()
{
    if (instance_ == nullptr)
    {
        return;
    }

    // Read all available characters from FIFO
    while (uart_is_readable(instance_->uart_))
    {
        char c = uart_getc(instance_->uart_);
        instance_->processReceivedChar(c);
    }
}

void BluetoothInterface::processReceivedChar(char c)
{
    last_char_ = c;

    // Echo received character back for debugging
    uart_putc(uart_, c);

    // Ignore line endings
    if (c == '\n' || c == '\r')
        return;

    // Parse single-character commands (case-insensitive)
    switch (c | 0x20)  // Convert to lowercase via bitwise OR
    {
        case 's': pending_command_ = Command::START;   break;
        case 'h': pending_command_ = Command::HALT;    break;
        case 'r': pending_command_ = Command::RESET;   break;
        case 'b': pending_command_ = Command::BATTERY; break;
        default:  pending_command_ = Command::UNKNOWN; break;
    }
    command_ready_ = true;
}
