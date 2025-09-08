#ifndef ENCODER_H
#define ENCODER_H

#include "../library/QuadratureEncoder.h"

class Encoder {
 public:
  Encoder(int gpioEncoderPinOne);
  ~Encoder();

  int getCurrentEncoderTickCount();

 private:
  void loadPIOProgram();

  const PIO pioInstance = pio0;
  uint pioStateMachine;
};
#endif