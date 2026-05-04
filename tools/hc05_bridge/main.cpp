/**
 * tools/hc05_bridge/main.cpp
 *
 * USB <-> UART0 pass-through for reconfiguring HC-05 Bluetooth modules.
 *
 * Why this exists:
 *   The Jurababa firmware drives HC-05 at 115200 baud (see firmware/main.cpp).
 *   HC-05 modules ship at 9600 baud and must be reprogrammed once via AT
 *   commands. AT mode is always 38400 baud regardless of the module's SPP
 *   baud, and the only way to reach it without a USB-TTL adapter is to use
 *   the Pico itself as a serial bridge.
 *
 * Wiring (same as normal Jurababa operation -- no rewiring needed):
 *   HC-05 VCC  -> Pico 5V (VBUS, pin 40) or 3.3V (pin 36)
 *   HC-05 GND  -> Pico GND
 *   HC-05 TXD  -> Pico GP13 (PIN_BT_RX in firmware/config/pins.h)
 *   HC-05 RXD  -> Pico GP12 (PIN_BT_TX)
 *
 * Procedure: see tools/hc05_bridge/README.md
 */

#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// Match firmware/config/pins.h. Duplicated intentionally so this utility has
// no dependency on the main firmware tree.
#define PIN_BT_TX 12 // Pico TX -> HC-05 RXD
#define PIN_BT_RX 13 // Pico RX <- HC-05 TXD

// HC-05 full AT mode is locked at 38400 in module firmware. This is
// independent of AT+UART (which sets the SPP communication baud).
#define AT_MODE_BAUD 38400

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB-CDC enumeration

    uart_init(uart0, AT_MODE_BAUD);
    gpio_set_function(PIN_BT_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_BT_RX, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    uart_set_hw_flow(uart0, false, false);

    printf("\n========================================\n");
    printf("  HC-05 USB <-> UART0 bridge @ %d baud\n", AT_MODE_BAUD);
    printf("========================================\n");
    printf("  HC-05 LED slow blink (~1 Hz) = full AT mode ready.\n");
    printf("  Set terminal line endings to CR+LF.\n");
    printf("  Try:\n");
    printf("    AT\n");
    printf("    AT+UART=115200,0,0\n");
    printf("    AT+UART?\n");
    printf("========================================\n\n");

    while (true)
    {
        // USB -> HC-05
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT)
        {
            uart_putc_raw(uart0, static_cast<char>(c));
        }

        // HC-05 -> USB (drain everything so multi-byte responses arrive in order)
        while (uart_is_readable(uart0))
        {
            putchar(uart_getc(uart0));
        }
    }
}
