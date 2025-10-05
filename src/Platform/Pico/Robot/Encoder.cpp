#include "../../../Include/Platform/Pico/Robot/Encoder.h"

// Constructor initializes the encoder using PIO and GPIO pin.
// Assumes channel B is the next sequential pin.
Encoder::Encoder(PIO pioInstance, int gpioPin, bool invertDirection) : pioInstance(pioInstance), offsetTicks(0), invertDirection(invertDirection) {
  stateMachine = pio_claim_unused_sm(pioInstance, true);
  loadPIOProgram(pioInstance);
  quadrature_encoder_program_init(pioInstance, stateMachine, gpioPin, 0);
}

// Ensure the quadrature decoder PIO program is only loaded once.
void Encoder::loadPIOProgram(PIO pioInstance) {
  static bool programLoaded = false;
  if (!programLoaded) {
    pio_add_program(pioInstance, &quadrature_encoder_program);
    programLoaded = true;
  }
}

// Return tick count adjusted for software reset.
int Encoder::getTickCount() const {
  int rawTicks = quadrature_encoder_get_count(pioInstance, stateMachine) * (invertDirection ? -1 : 1);
  return rawTicks - offsetTicks;
}

// Reset encoder by storing current count as offset.
void Encoder::reset() {
  offsetTicks = quadrature_encoder_get_count(pioInstance, stateMachine);
}