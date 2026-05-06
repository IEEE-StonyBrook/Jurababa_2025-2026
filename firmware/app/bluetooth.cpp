#include "app/bluetooth.h"

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/critical_section.h"
#include "pico/time.h"

Bluetooth* Bluetooth::instance_                         = nullptr;
uint8_t    Bluetooth::tx_ring_[Bluetooth::TX_RING_SIZE] = {};

struct Bluetooth::TxLock
{
    critical_section_t section;
};

namespace
{
char toUpperAscii(char c)
{
    if (c >= 'a' && c <= 'z')
        return static_cast<char>(c - 'a' + 'A');
    return c;
}

bool printableAscii(char c)
{
    return c >= 0x20 && c <= 0x7e;
}

bool shortcutChar(char c)
{
    return c == 's' || c == 'h' || c == 'r' || c == 'b';
}

Bluetooth::Command commandFromShortcut(char c)
{
    switch (c)
    {
        case 's':
            return Bluetooth::Command::START;
        case 'h':
            return Bluetooth::Command::HALT;
        case 'r':
            return Bluetooth::Command::RESET;
        case 'b':
            return Bluetooth::Command::BATTERY;
        default:
            return Bluetooth::Command::UNKNOWN;
    }
}
} // namespace

Bluetooth::Bluetooth(uart_inst_t* uart, uint32_t baud_rate, uint8_t tx_pin, uint8_t rx_pin)
    : uart_(uart != nullptr ? uart : uart0), baud_rate_(baud_rate), tx_pin_(tx_pin),
      rx_pin_(rx_pin), pending_command_(Command::NONE), command_ready_(false)
{
    static TxLock tx_lock_storage;
    tx_lock_ = &tx_lock_storage;
    critical_section_init(&tx_lock_->section);
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

void Bluetooth::writeBytes(const uint8_t* data, std::size_t length)
{
    if (data == nullptr)
        return;

    for (std::size_t i = 0; i < length; ++i)
        enqueueByte(data[i]);
}

void Bluetooth::drain()
{
    while (uart_is_writable(uart_))
    {
        uint8_t byte = 0;

        critical_section_enter_blocking(&tx_lock_->section);
        if (tx_tail_ == tx_head_)
        {
            critical_section_exit(&tx_lock_->section);
            break;
        }

        byte     = tx_ring_[tx_tail_];
        tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % TX_RING_SIZE);
        critical_section_exit(&tx_lock_->section);

        uart_putc_raw(uart_, byte);
    }
}

Bluetooth::Diagnostics Bluetooth::diagnostics() const
{
    critical_section_enter_blocking(&tx_lock_->section);
    Diagnostics diag = {ringDepthLocked(), max_tx_depth_,      dropped_tx_bytes_,
                        rxLineDepth(),     max_rx_line_depth_, dropped_rx_lines_,
                        dropped_rx_chars_, pending_shortcut_};
    critical_section_exit(&tx_lock_->section);
    return diag;
}

bool Bluetooth::hasCommand()
{
    uint32_t flags = save_and_disable_interrupts();
    bool     ready = command_ready_ || promotePendingShortcut();
    restore_interrupts(flags);
    return ready;
}

Bluetooth::Command Bluetooth::command()
{
    uint32_t flags = save_and_disable_interrupts();
    if (!(command_ready_ || promotePendingShortcut()))
    {
        restore_interrupts(flags);
        return Command::NONE;
    }

    Command cmd      = pending_command_;
    command_ready_   = false;
    pending_command_ = Command::NONE;
    restore_interrupts(flags);
    return cmd;
}

bool Bluetooth::hasLine() const
{
    return rx_line_tail_ != rx_line_head_;
}

bool Bluetooth::readLine(char* out, std::size_t out_length)
{
    if (out == nullptr || out_length == 0 || !hasLine())
        return false;

    const RxLine& line = rx_lines_[rx_line_tail_];
    std::size_t   len  = line.length;
    if (len >= out_length)
        len = out_length - 1;

    std::memcpy(out, line.text, len);
    out[len]      = '\0';
    rx_line_tail_ = static_cast<uint16_t>((rx_line_tail_ + 1) % RX_LINE_QUEUE_SIZE);
    return true;
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
    {
        if (pending_shortcut_)
        {
            appendRxChar(toUpperAscii(pending_shortcut_char_));
            pending_shortcut_ = false;
        }
        enqueueRxLine();
        return;
    }

    if (c == '\b' || c == 127)
    {
        if (pending_shortcut_)
        {
            pending_shortcut_ = false;
            return;
        }
        if (rx_line_length_ > 0)
            --rx_line_length_;
        return;
    }

    if (!printableAscii(c))
        return;

    if (pending_shortcut_)
    {
        appendRxChar(toUpperAscii(pending_shortcut_char_));
        pending_shortcut_ = false;
    }

    if (rx_line_length_ == 0 && shortcutChar(c))
    {
        pending_shortcut_      = true;
        pending_shortcut_char_ = c;
        pending_shortcut_us_   = time_us_32();
        return;
    }

    appendRxChar(toUpperAscii(c));
}

void Bluetooth::enqueueByte(uint8_t byte)
{
    critical_section_enter_blocking(&tx_lock_->section);

    uint16_t next = static_cast<uint16_t>((tx_head_ + 1) % TX_RING_SIZE);
    if (next == tx_tail_)
    {
        ++dropped_tx_bytes_;
        critical_section_exit(&tx_lock_->section);
        return;
    }

    tx_ring_[tx_head_] = byte;
    tx_head_           = next;

    uint16_t depth = ringDepthLocked();
    if (depth > max_tx_depth_)
        max_tx_depth_ = depth;

    critical_section_exit(&tx_lock_->section);
}

void Bluetooth::appendRxChar(char c)
{
    if (rx_line_length_ >= RX_LINE_BUFFER_SIZE - 1)
    {
        ++dropped_rx_chars_;
        return;
    }

    rx_line_buffer_[rx_line_length_++] = c;
    rx_line_buffer_[rx_line_length_]   = '\0';
}

void Bluetooth::enqueueRxLine()
{
    if (rx_line_length_ == 0)
        return;

    const char* text   = rx_line_buffer_;
    uint16_t    length = rx_line_length_;

    if ((length == 1 && text[0] == 'G') || (length == 5 && std::memcmp(text, "START", 5) == 0))
    {
        setPendingCommand(Command::START);
        rx_line_length_    = 0;
        rx_line_buffer_[0] = '\0';
        return;
    }

    if ((length == 1 && (text[0] == 'H' || text[0] == 'X')) ||
        (length == 4 && std::memcmp(text, "HALT", 4) == 0) ||
        (length == 4 && std::memcmp(text, "STOP", 4) == 0))
    {
        setPendingCommand(Command::HALT);
        rx_line_length_    = 0;
        rx_line_buffer_[0] = '\0';
        return;
    }

    if (length == 5 && std::memcmp(text, "RESET", 5) == 0)
    {
        setPendingCommand(Command::RESET);
        rx_line_length_    = 0;
        rx_line_buffer_[0] = '\0';
        return;
    }

    uint16_t next = static_cast<uint16_t>((rx_line_head_ + 1) % RX_LINE_QUEUE_SIZE);
    if (next == rx_line_tail_)
    {
        ++dropped_rx_lines_;
        rx_line_length_    = 0;
        rx_line_buffer_[0] = '\0';
        return;
    }

    RxLine& line = rx_lines_[rx_line_head_];
    std::memcpy(line.text, rx_line_buffer_, length);
    line.text[length] = '\0';
    line.length       = length;
    rx_line_head_     = next;

    uint16_t depth = rxLineDepth();
    if (depth > max_rx_line_depth_)
        max_rx_line_depth_ = depth;

    rx_line_length_    = 0;
    rx_line_buffer_[0] = '\0';
}

bool Bluetooth::setPendingCommand(Command command)
{
    if (command_ready_)
    {
        ++dropped_rx_lines_;
        return false;
    }

    pending_command_ = command;
    command_ready_   = true;
    return true;
}

bool Bluetooth::promotePendingShortcut()
{
    if (!pending_shortcut_)
        return false;

    uint32_t elapsed_us = time_us_32() - pending_shortcut_us_;
    if (elapsed_us < SHORTCUT_DELAY_US)
        return false;

    Command command   = commandFromShortcut(pending_shortcut_char_);
    pending_shortcut_ = false;
    return setPendingCommand(command);
}

uint16_t Bluetooth::ringDepthLocked() const
{
    if (tx_head_ >= tx_tail_)
        return static_cast<uint16_t>(tx_head_ - tx_tail_);
    return static_cast<uint16_t>(TX_RING_SIZE - tx_tail_ + tx_head_);
}

uint16_t Bluetooth::rxLineDepth() const
{
    if (rx_line_head_ >= rx_line_tail_)
        return static_cast<uint16_t>(rx_line_head_ - rx_line_tail_);
    return static_cast<uint16_t>(RX_LINE_QUEUE_SIZE - rx_line_tail_ + rx_line_head_);
}
