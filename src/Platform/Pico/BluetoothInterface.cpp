#include "Platform/Pico/BluetoothInterface.h"

#include <cstring>

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
    // Initialize UART with specified baud rate
    uart_init(uart_, baud_rate_);

    // Configure GPIO pins for UART function
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);

    // Set UART format: 8 data bits, 1 stop bit, no parity
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);

    // Enable FIFO for better performance
    uart_set_fifo_enabled(uart_, true);

    // Determine IRQ number based on UART instance
    int uart_irq = (uart_ == uart0) ? UART0_IRQ : UART1_IRQ;

    // Set up interrupt handler
    irq_set_exclusive_handler(uart_irq, rxInterruptHandler);
    irq_set_enabled(uart_irq, true);

    // Enable RX interrupt only (TX handled synchronously)
    uart_set_irq_enables(uart_, true, false);
}

void BluetoothInterface::write(const std::string& data)
{
    uart_puts(uart_, data.c_str());
}

void BluetoothInterface::write(const char* data)
{
    uart_puts(uart_, data);
}

void BluetoothInterface::writeBytes(const uint8_t* data, size_t length)
{
    uart_write_blocking(uart_, data, length);
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

    // Parse single-character commands
    switch (c)
    {
        case 's':
        case 'S':
            pending_command_ = Command::START;
            command_ready_   = true;
            break;

        case 'h':
        case 'H':
            pending_command_ = Command::HALT;
            command_ready_   = true;
            break;

        case 'r':
        case 'R':
            pending_command_ = Command::RESET;
            command_ready_   = true;
            break;

        case 'b':
        case 'B':
            pending_command_ = Command::BATTERY;
            command_ready_   = true;
            break;

        case '\n':
        case '\r':
            // Ignore line endings
            break;

        default:
            pending_command_ = Command::UNKNOWN;
            command_ready_   = true;
            break;
    }
}
