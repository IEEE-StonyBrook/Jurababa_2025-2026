#include "../../../Include/Platform/Pico/Robot/Encoder.h"
#ifdef USE_MULTICORE_SENSORS
#include "../../../Include/Platform/Pico/MulticoreSensors.h"
#endif

// Constructor initializes the encoder using PIO and GPIO pin.
// Assumes channel B is the next sequential pin.
Encoder::Encoder(PIO pioInstance, int gpioPin, bool invertDirection) : pioInstance(pioInstance), gpioPin(gpioPin), offsetTicks(0), invertDirection(invertDirection) {
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
//   // If multicore sensor hub is enabled and we're not the publisher core,
//   // read the latest snapshot from the shared hub instead of probing the
//   // hardware. This keeps publishing limited to the top-level publisher in
//   // `main.cpp` per project convention.
// #ifdef USE_MULTICORE_SENSORS
//   // On the Pico, core 0 is typically the consumer. If this is the
//   // publisher core (core 1) we still read the hardware directly.
//   if (multicore_get_core_num() == 1) {
//     return quadrature_encoder_get_count(pioInstance, pioStateMachine);
//   }

//   MulticoreSensorData s = {};
//   MulticoreSensorHub::snapshot(s);

//   // Decide which encoder field to return based on gpio pin. This is a
//   // heuristic: maintain the previous behavior where instances default to
//   // left if ambiguous.
//   if (gpioEncoderPinOne == 20) {
//     return s.left_encoder_count;
//   } else if (gpioEncoderPinOne == 7) {
//     return s.right_encoder_count;
//   }

//   // Fallback to front/left choice: return left by default.
//   return s.left_encoder_count;
// #else
//   // Multicore disabled: read hardware directly.
//   return quadrature_encoder_get_count(pioInstance, pioStateMachine);
// #endif
  int rawTicks = quadrature_encoder_get_count(pioInstance, stateMachine) * (invertDirection ? -1 : 1);
  return rawTicks - offsetTicks;
}

// Reset encoder by storing current count as offset.
void Encoder::reset() {
  LOG_DEBUG("Resetting encoder...");
  offsetTicks = quadrature_encoder_get_count(pioInstance, stateMachine);
}