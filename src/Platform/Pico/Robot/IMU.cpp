#include "../../../Include/Platform/Pico/Robot/IMU.h"

#include <cmath>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#ifdef USE_MULTICORE_SENSORS
#include "../../../Include/Platform/Pico/MulticoreSensors.h"
#endif
#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_IRQ UART1_IRQ

#include "../../../Include/Common/LogSystem.h"

IMU* IMU::imuInstance = nullptr;

IMU::IMU(int uartRXPin) : uartRXPin(uartRXPin), IMUBufferIndex(0) {
  robotYawNeg180To180Degrees = 0.0f;
  resetOffSet = 0.0f;
  setUpIMUCommunication();
  setUpIMUInterrupts();
  LOG_DEBUG("IMU Initialized");
}

void IMU::setUpIMUCommunication() {
  imuInstance = this;
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function((uint)uartRXPin, GPIO_FUNC_UART);
  uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(UART_ID, true);   //was false
}

void IMU::setUpIMUInterrupts() {
  uart_set_irq_enables(UART_ID, true, false);
  irq_set_exclusive_handler(UART_IRQ, IMU::imuInterruptHandler);
  irq_set_enabled(UART_IRQ, true);
  LOG_DEBUG("IMU Interrupts Set");
}

void IMU::imuInterruptHandler() {
  //LOG_DEBUG("In Handler");
  if (imuInstance != nullptr) {
    imuInstance->processIMURXInterruptData();
    //LOG_DEBUG("Handler reached.");
  }
  //LOG_DEBUG("no handler");
}

void IMU::processIMURXInterruptData() {
  //volatile int IMUBufferIndex = 0;
  //LOG_DEBUG("IMU RX Interrupt Start");
  while (uart_is_readable(UART_ID)) {
    //LOG_DEBUG("IMU RX Interrupt");
    uint8_t character = uart_getc(UART_ID);
    IMUBufferForYaw[IMUBufferIndex] = character;
    if (IMUBufferIndex == 18) {
      convertPacketDataToUsableYaw();
    }
    IMUBufferIndex++;
  }
}

void IMU::convertPacketDataToUsableYaw() {
  IMUBufferIndex = 0;
  LOG_DEBUG("afteryawfunction");
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
    LOG_DEBUG("IMU Yaw: " + std::to_string(robotYawNeg180To180Degrees));

    // When multicore hub is enabled the top-level publisher in `main.cpp`
    // should be responsible for publishing. Here we avoid publishing from
    // the interrupt and instead let consumers snapshot the hub. If running
    // as the publisher core (core 1) we still publish so the hub receives
    // fresh data.
// #ifdef USE_MULTICORE_SENSORS
//     if (multicore_get_core_num() == 1) {
//       MulticoreSensorData s = {};
//       s.imu_yaw = robotYawNeg180To180Degrees - resetOffSet;
//       s.timestamp_ms = to_ms_since_boot(get_absolute_time());
//       MulticoreSensorHub::publish(s);
//     }
// #endif
  } else {
        //LOG_DEBUG("IMU checksum error: calculated %d, expected %d\n", sumOfByteData, IMUBufferForYaw[18]);
    }
}

float IMU::getIMUYawDegreesNeg180ToPos180() {
// #ifdef USE_MULTICORE_SENSORS
//   // If multicore is enabled and this is the consumer core (core 0), read
//   // the latest snapshot instead of returning the locally-updated value.
//   if (multicore_get_core_num() == 0) {
//     MulticoreSensorData s = {};
//     MulticoreSensorHub::snapshot(s);
//     return s.imu_yaw - resetOffSet;
//   }
// #endif

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