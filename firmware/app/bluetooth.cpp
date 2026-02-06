#include "app/bluetooth.h"

#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

Bluetooth* Bluetooth::instance_ = nullptr;

Bluetooth::Bluetooth(uart_inst_t* uart, uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin)
    : uart_(uart), baud_rate_(baud_rate), tx_pin_(tx_pin), rx_pin_(rx_pin),
      pending_command_(Command::NONE), last_char_(0), command_ready_(false)
{
    instance_ = this;
}

void Bluetooth::init()
{
    printf("[BT] Starting Bluetooth init...\n");
    printf("[BT] UART: %s, Baud: %lu, TX: %d, RX: %d\n", (uart_ == uart0) ? "uart0" : "uart1",
           baud_rate_, tx_pin_, rx_pin_);

    uint actual_baud = uart_init(uart_, baud_rate_);
    printf("[BT] Actual baud rate: %u\n", actual_baud);

    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);

    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_, true);

    int uart_irq = (uart_ == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, rxInterruptHandler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(uart_, true, false);

    uart_puts(uart_, "BT_INIT_OK\r\n");
    printf("[BT] Bluetooth init complete!\n");
}

void Bluetooth::write(const std::string& data)
{
    uart_puts(uart_, data.c_str());
}

void Bluetooth::write(const char* data)
{
    uart_puts(uart_, data);
}

void Bluetooth::writeBytes(const uint8_t* data, size_t length)
{
    uart_write_blocking(uart_, data, length);
}

bool Bluetooth::hasCommand() const
{
    return command_ready_;
}

Bluetooth::Command Bluetooth::command()
{
    if (!command_ready_)
        return Command::NONE;

    Command cmd      = pending_command_;
    command_ready_   = false;
    pending_command_ = Command::NONE;
    return cmd;
}

char Bluetooth::lastChar() const
{
    return last_char_;
}

bool Bluetooth::txReady() const
{
    return uart_is_writable(uart_);
}

void Bluetooth::rxInterruptHandler()
{
    if (instance_ == nullptr)
        return;

    while (uart_is_readable(instance_->uart_))
    {
        char c = uart_getc(instance_->uart_);
        instance_->processChar(c);
    }
}

void Bluetooth::processChar(char c)
{
    last_char_ = c;
    uart_putc(uart_, c); // Echo

    if (c == '\n' || c == '\r')
        return;

    // Parse single-character commands (case-insensitive)
    switch (c | 0x20)
    {
        case 's':
            pending_command_ = Command::START;
            break;
        case 'h':
            pending_command_ = Command::HALT;
            break;
        case 'r':
            pending_command_ = Command::RESET;
            break;
        case 'b':
            pending_command_ = Command::BATTERY;
            break;
        default:
            pending_command_ = Command::UNKNOWN;
            break;
    }
    command_ready_ = true;
}
