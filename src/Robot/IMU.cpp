#include "IMU.h"

#include <cmath>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_IRQ UART1_IRQ

std::array<uint8_t, 19> IMU::IMUBufferForYaw = {};
float IMU::robotYawNeg180To180Degrees = 0.0f;
float IMU::resetOffSet = 0.0f;

IMU::IMU(int uartRXPin) : uartRXPin(uartRXPin) {
  setUpIMUCommunication();
  setUpIMUInterrupts();
}

void IMU::setUpIMUCommunication() {
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function((uint)uartRXPin, GPIO_FUNC_UART);
  uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(UART_ID, false);
}

void IMU::setUpIMUInterrupts() {
  irq_set_exclusive_handler(UART_IRQ, IMU::processIMURXInterruptData);
  irq_set_enabled(UART_IRQ, true);
}

void IMU::processIMURXInterruptData() {
  volatile int IMUBufferIndex = 0;
  while (uart_is_readable(UART_ID)) {
    uint8_t character = uart_getc(UART_ID);
    IMUBufferForYaw[IMUBufferIndex] = character;
    if (IMUBufferIndex == 18) {
      convertPacketDataToUsableYaw();
    }
    IMUBufferIndex++;
  }
}

void IMU::convertPacketDataToUsableYaw() {
  uint8_t sumOfByteData = 0;
  for (int i = 2; i < 15; i++) {
    sumOfByteData += IMUBufferForYaw[i];
  }

  if (sumOfByteData == IMUBufferForYaw[18]) {
    int16_t currentYaw = (IMUBufferForYaw[4] << 8) | IMUBufferForYaw[3];
    float negOrPos = (currentYaw / fabs(currentYaw));
    float currentYaw0To360Degrees =
        fmod((fabs((float)currentYaw) / 100.0f), 360.0f) * negOrPos;
    robotYawNeg180To180Degrees = currentYaw0To360Degrees - 180.0f;
  }
}

float IMU::getIMUYawDegreesNeg180ToPos180() {
  return robotYawNeg180To180Degrees - resetOffSet;
}

void IMU::resetIMUYawToZero() {
  resetOffSet = getIMUYawDegreesNeg180ToPos180();
}

float IMU::getNewYawAfterAddingDegrees(float degreesToAdd) {
  float negOrPos = (degreesToAdd / fabs(degreesToAdd));
  degreesToAdd = fmod(fabs(degreesToAdd), 360.0f) * negOrPos;

  float newYaw = getIMUYawDegreesNeg180ToPos180() + degreesToAdd;

  // Determines if the robot needs to rotate left/right to turn efficiently.
  if (newYaw > 180.0f) {
    newYaw = -360.0f + newYaw;
  } else if (newYaw < -180.0f) {
    newYaw = 360.0f + newYaw;
  }

  return newYaw;
}