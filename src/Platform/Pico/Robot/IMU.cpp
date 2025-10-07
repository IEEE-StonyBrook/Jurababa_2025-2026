#include "../../../Include/Platform/Pico/Robot/IMU.h"

#ifdef USE_MULTICORE_SENSORS
#include "../../../Include/Platform/Pico/MulticoreSensors.h"
#endif

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
  uart_init(IMU_UART_ID, IMU_BAUD_RATE);
  gpio_set_function((uint)uartRXPin, GPIO_FUNC_UART);
  uart_set_baudrate(IMU_UART_ID, IMU_BAUD_RATE);
  uart_set_hw_flow(IMU_UART_ID, false, false);
  uart_set_format(IMU_UART_ID, IMU_DATA_BITS, IMU_STOP_BITS, IMU_PARITY);
  uart_set_fifo_enabled(IMU_UART_ID, true);  // was false
}

void IMU::setUpIMUInterrupts() {
  uart_set_irq_enables(IMU_UART_ID, true, false);
  irq_set_exclusive_handler(IMU_UART_IRQ, IMU::imuInterruptHandler);
  irq_set_enabled(IMU_UART_IRQ, true);
  LOG_DEBUG("IMU Interrupts Set");
}

void IMU::imuInterruptHandler() {
  // LOG_DEBUG("In Handler");
  if (imuInstance != nullptr) {
    imuInstance->processIMURXInterruptData();
    // LOG_DEBUG("Handler reached.");
  }
  // LOG_DEBUG("no handler");
}

void IMU::processIMURXInterruptData() {
  while (uart_is_readable(IMU_UART_ID)) {
    uint8_t byte = uart_getc(IMU_UART_ID);

    // Store byte in buffer.
    IMUBufferForYaw[IMUBufferIndex++] = byte;

    // If we haven't yet filled a full packet, continue.
    if (IMUBufferIndex < IMU_PACKET_LEN) continue;

    // --- Packet full, validate header ---
    if (IMUBufferForYaw[IMU_IDX_HDR0] != IMU_HDR0) {
      // Shift buffer left by 1 to resync and continue.
      for (int i = 1; i < IMU_PACKET_LEN; i++) {
        IMUBufferForYaw[i - 1] = IMUBufferForYaw[i];
      }
      IMUBufferIndex = IMU_PACKET_LEN - 1;  // Expect next byte to complete.
      continue;
    }

    // Valid header found → parse the packet.
    convertPacketDataToUsableYaw();

    // Reset index for next packet.
    IMUBufferIndex = 0;
  }
}


void IMU::convertPacketDataToUsableYaw() {
  IMUBufferIndex = 0;

  // Check for valid starter bytes
  if (IMUBufferForYaw[IMU_IDX_HDR0] != IMU_HDR0) {
    LOG_DEBUG("Invalid HDR0: " + std::to_string(IMUBufferForYaw[IMU_IDX_HDR0]));
    return;
  }

  uint8_t sumOfByteData = 0;
  for (int i = 2; i < 15; i++) {
    sumOfByteData += IMUBufferForYaw[i];
  }

  if (sumOfByteData == IMUBufferForYaw[18]) {
    // Combine two bytes into a signed 16-bit yaw value
    int16_t currentYaw = (IMUBufferForYaw[4] << 8) | IMUBufferForYaw[3];
    
    // Convert to degrees (0–360 or negative)
    float currentYawDegrees = static_cast<float>(currentYaw) / 100.0f;

    // Wrap into [-180, 180]
    float wrappedYaw = fmod(currentYawDegrees, 360.0f);
    if (wrappedYaw > 180.0f)
      wrappedYaw -= 360.0f;
    else if (wrappedYaw < -180.0f)
      wrappedYaw += 360.0f;

    robotYawNeg180To180Degrees = wrappedYaw;

    LOG_DEBUG("IMU Yaw: " + std::to_string(robotYawNeg180To180Degrees));
  } else {
    // LOG_DEBUG("IMU checksum error: calculated %d, expected %d\n",
    // sumOfByteData, IMUBufferForYaw[18]);
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