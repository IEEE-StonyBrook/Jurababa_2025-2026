#include "../../../Include/Platform/Pico/Robot/Encoder.h"
#ifdef USE_MULTICORE_SENSORS
#include "../../../Include/Platform/Pico/MulticoreSensors.h"
#endif

// Assumes pin two is next pin.
Encoder::Encoder(int gpioEncoderPinOne) : gpioEncoderPinOne(gpioEncoderPinOne) {
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
  // If multicore sensor hub is enabled and we're not the publisher core,
  // read the latest snapshot from the shared hub instead of probing the
  // hardware. This keeps publishing limited to the top-level publisher in
  // `main.cpp` per project convention.
#ifdef USE_MULTICORE_SENSORS
  // On the Pico, core 0 is typically the consumer. If this is the
  // publisher core (core 1) we still read the hardware directly.
  if (multicore_get_core_num() == 1) {
    return quadrature_encoder_get_count(pioInstance, pioStateMachine);
  }

  MulticoreSensorData s = {};
  MulticoreSensorHub::snapshot(s);

  // Decide which encoder field to return based on gpio pin. This is a
  // heuristic: maintain the previous behavior where instances default to
  // left if ambiguous.
  if (gpioEncoderPinOne == 20) {
    return s.left_encoder_count;
  } else if (gpioEncoderPinOne == 7) {
    return s.right_encoder_count;
  }

  // Fallback to front/left choice: return left by default.
  return s.left_encoder_count;
#else
  // Multicore disabled: read hardware directly.
  return quadrature_encoder_get_count(pioInstance, pioStateMachine);
#endif
}