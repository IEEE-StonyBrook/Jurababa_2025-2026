#include "app/bluetooth.h"

#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

Bluetooth* Bluetooth::instance_                         = nullptr;
uint8_t    Bluetooth::tx_ring_[Bluetooth::TX_RING_SIZE] = {};

Bluetooth::Bluetooth(uart_inst_t* uart, uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin)
    : uart_(uart), baud_rate_(baud_rate), tx_pin_(tx_pin), rx_pin_(rx_pin),
      pending_command_(Command::NONE), command_ready_(false)
{
    critical_section_init(&tx_lock_);
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

    write("BT_INIT_OK\r\n");
    drain();
    printf("[BT] Bluetooth init complete!\n");
}

void Bluetooth::write(const std::string& data)
{
    writeBytes(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

void Bluetooth::write(const char* data)
{
    if (data == nullptr)
        return;

    writeBytes(reinterpret_cast<const uint8_t*>(data), std::strlen(data));
}

void Bluetooth::writeBytes(const uint8_t* data, size_t length)
{
    if (data == nullptr)
        return;

    for (size_t i = 0; i < length; ++i)
        enqueueByte(data[i]);
}

void Bluetooth::drain()
{
    while (uart_is_writable(uart_))
    {
        uint8_t byte = 0;

        critical_section_enter_blocking(&tx_lock_);
        if (tx_tail_ == tx_head_)
        {
            critical_section_exit(&tx_lock_);
            break;
        }

        byte     = tx_ring_[tx_tail_];
        tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % TX_RING_SIZE);
        critical_section_exit(&tx_lock_);

        uart_putc_raw(uart_, byte);
    }
}

Bluetooth::Diagnostics Bluetooth::diagnostics() const
{
    critical_section_enter_blocking(&tx_lock_);
    Diagnostics diag = {ringDepthLocked(), max_tx_depth_, dropped_tx_bytes_};
    critical_section_exit(&tx_lock_);
    return diag;
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

void Bluetooth::enqueueByte(uint8_t byte)
{
    critical_section_enter_blocking(&tx_lock_);

    uint16_t next = static_cast<uint16_t>((tx_head_ + 1) % TX_RING_SIZE);
    if (next == tx_tail_)
    {
        ++dropped_tx_bytes_;
        critical_section_exit(&tx_lock_);
        return;
    }

    tx_ring_[tx_head_] = byte;
    tx_head_           = next;

    uint16_t depth = ringDepthLocked();
    if (depth > max_tx_depth_)
        max_tx_depth_ = depth;

    critical_section_exit(&tx_lock_);
}

uint16_t Bluetooth::ringDepthLocked() const
{
    if (tx_head_ >= tx_tail_)
        return static_cast<uint16_t>(tx_head_ - tx_tail_);
    return static_cast<uint16_t>(TX_RING_SIZE - tx_tail_ + tx_head_);
}
