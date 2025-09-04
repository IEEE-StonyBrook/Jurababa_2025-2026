#include "Encoder.h"

// Assumes pin two is next pin.
Encoder::Encoder(int gpioEncoderPinOne) {
  pioStateMachine = pio_claim_unused_sm(pioInstance, true);
  
  loadPIOProgram();
  quadrature_encoder_program_init(pioInstance, pioStateMachine,
                                  gpioEncoderPinOne, 0);
};

// Prevents re-adding program for other encoder instantiation.
void Encoder::loadPIOProgram() {
  static bool pioProgramLoaded = false;
  if (!pioProgramLoaded) {
    pio_add_program(pioInstance, &quadrature_encoder_program);
    pioProgramLoaded = true;
  }
}

int Encoder::getCurrentEncoderTickCount() {
  return quadrature_encoder_get_count(pioInstance, pioStateMachine);
}