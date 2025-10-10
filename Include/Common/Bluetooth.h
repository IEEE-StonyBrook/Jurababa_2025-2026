#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <string>

#include "../Platform/Pico/Config.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

class Bluetooth {
 public:
  Bluetooth(uart_inst_t* uart_id = uart0, uint tx_pin = 16, uint rx_pin = 17,
            uint baud_rate = 9600);

  void init();
  void sendChar(char c);
  void sendString(const std::string& str);
  bool available();
  char readChar();
  std::string readLine();

 private:
  uart_inst_t* uart_id;
  uint tx_pin;
  uint rx_pin;
  uint baud_rate;
};

#endif  // BLUETOOTH_H