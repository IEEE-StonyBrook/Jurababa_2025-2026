#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>

#include "../../../External/QuadratureEncoder.h"
#include "../../../Include/Common/LogSystem.h"
#include "../Config.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

class Encoder
{
  public:
    // Constructor initializes the encoder on the given PIO instance and GPIO pin.
    Encoder(PIO pioInstance, int gpioPin, bool invertDirection = false);

    // Reset the encoder tick count to zero.
    void reset();

    // Return the current tick count.
    int getTickCount() const;

  private:
    // Load the quadrature decoder PIO program once into hardware.
    void loadPIOProgram(PIO pioInstance);

    const PIO pioInstance;     // PIO block used by this encoder. = pio0;
    uint      stateMachine;    // State machine index for the PIO program.
    int       offsetTicks;     // Offset to allow software reset of tick count.
    bool      invertDirection; // Whether to invert the tick count direction.
    // Remember which GPIO pin this encoder instance is using so consumers can
    // decide which field of the MulticoreSensorData to return (left vs right).
    int gpioPin;
};

#endif