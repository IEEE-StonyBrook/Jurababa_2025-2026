#include "../../Include/Common/Bluetooth.h"

Bluetooth::Bluetooth(uart_inst_t* uart_id, uint tx_pin, uint rx_pin,
                     uint baud_rate)
    : uart_id(uart_id), tx_pin(tx_pin), rx_pin(rx_pin), baud_rate(baud_rate) {}

void Bluetooth::init() {
  // Init UART
  uart_init(uart_id, baud_rate);

  // Set GPIO function
  gpio_set_function(tx_pin, GPIO_FUNC_UART);
  gpio_set_function(rx_pin, GPIO_FUNC_UART);

  // Configure UART: 8 data bits, 1 stop bit, no parity
  uart_set_hw_flow(uart_id, false, false);
  uart_set_format(uart_id, 8, 1, UART_PARITY_NONE);

  // Enable FIFO
  uart_set_fifo_enabled(uart_id, true);
}

void Bluetooth::sendChar(char c) { uart_putc_raw(uart_id, c); }

void Bluetooth::sendString(const std::string& str) {
  for (char c : str) {
    uart_putc_raw(uart_id, c);
  }
}

bool Bluetooth::available() { return uart_is_readable(uart_id); }

char Bluetooth::readChar() {
  if (uart_is_readable(uart_id)) {
    return uart_getc(uart_id);
  }
  return '\0';  // return null if nothing
}

std::string Bluetooth::readLine() {
  std::string result;
  while (true) {
    if (uart_is_readable(uart_id)) {
      char c = uart_getc(uart_id);
      if (c == '\n' || c == '\r') break;
      result.push_back(c);
    } else {
      sleep_ms(5);  // small wait to avoid busy loop
    }
  }
  return result;
}