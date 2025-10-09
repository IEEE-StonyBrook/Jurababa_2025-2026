#include "../../../Include/Platform/Pico/Robot/IMU.h"

#ifdef USE_MULTICORE_SENSORS
#include "../../../Include/Platform/Pico/MulticoreSensors.h"
#endif

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

// void IMU::processIMURXInterruptData() {
//   // volatile int IMUBufferIndex = 0;
//   // LOG_DEBUG("IMU RX Interrupt Start");
//   while (uart_is_readable(IMU_UART_ID)) {
//     // LOG_DEBUG("IMU RX Interrupt");
//     uint8_t character = uart_getc(IMU_UART_ID);
//     IMUBufferForYaw[IMUBufferIndex] = character;
//     if (IMUBufferIndex == 18) {
//       convertPacketDataToUsableYaw();
//     }
//     IMUBufferIndex++;
//   }

// }

void IMU::processIMURXInterruptData() {
  while (uart_is_readable(IMU_UART_ID)) {
    uint8_t character = uart_getc(IMU_UART_ID);

    // --- Header Resynchronization Check ---
    // If we're at the start of a new packet, make sure the first byte is
    // correct
    if (IMUBufferIndex == 0 && character != IMU_HDR0) {
      // Wait for correct header before starting to fill the buffer
      continue;
    }

    IMUBufferForYaw[IMUBufferIndex++] = character;

    // If we filled a full packet
    if (IMUBufferIndex >= IMU_PACKET_LEN) {
      // Optional: Verify header again (redundant safety)
      if (IMUBufferForYaw[IMU_IDX_HDR0] == IMU_HDR0) {
        convertPacketDataToUsableYaw();
      }

      // Reset buffer index for the next packet (good practice)
      IMUBufferIndex = 0;
    }
  }
}

void IMU::convertPacketDataToUsableYaw() {
  IMUBufferIndex = 0;

  uint8_t sumOfByteData = 0;
  for (int i = 2; i < 15; i++) {
    sumOfByteData += IMUBufferForYaw[i];
  }

  if (sumOfByteData == IMUBufferForYaw[18]) {
    int16_t currentYaw = (IMUBufferForYaw[4] << 8) | IMUBufferForYaw[3];

    // Convert raw to degrees (assuming 100 LSB/deg)
    float yawDegrees = static_cast<float>(currentYaw) / 100.0f;

    // Wrap to [0, 360)
    float yaw0To360 = fmodf(yawDegrees + 360.0f, 360.0f);

    // Wrap to [-180, 180)
    float yawNeg180To180 = fmodf(yawDegrees + 180.0f, 360.0f) - 180.0f;

    robotYawNeg180To180Degrees = yawNeg180To180;
    yawReady = true;

    // LOG_DEBUG("IMU Yaw: " + std::to_string(yawNeg180To180) + " deg");
  } else {
    // checksum failed, do nothing
  }
}

float IMU::getIMUYawDegreesNeg180ToPos180() {
  if (!yawReady) {
    // LOG_DEBUG("IMU yaw not ready yet, returning 0");
    return 0.0f;
  }
  return robotYawNeg180To180Degrees - resetOffSet;
}

void IMU::resetIMUYawToZero() {
  if (!yawReady) {
    // LOG_DEBUG("Cannot reset yaw - IMU not ready yet.");
    return;
  }
  // LOG_DEBUG("Resetting IMU yaw to zero...");
  float currentYaw = getIMUYawDegreesNeg180ToPos180();
  resetOffSet = currentYaw;
}

float IMU::getNewYawAfterAddingDegrees(float degreesToAdd) {
float negOrPos = (degreesToAdd == 0.0f) ? 0.0f : (degreesToAdd / fabs(degreesToAdd));
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