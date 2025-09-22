#ifndef ENCODER_H
#define ENCODER_H

#include "../../../External/QuadratureEncoder.h"

class Encoder {
 public:
  Encoder(int gpioEncoderPinOne);

  int getCurrentEncoderTickCount();

 private:
  void loadPIOProgram();

  const PIO pioInstance = pio0;
  uint pioStateMachine;
};
#endif