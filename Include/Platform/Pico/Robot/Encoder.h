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
  // Remember which GPIO pin this encoder instance is using so consumers can
  // decide which field of the MulticoreSensorData to return (left vs right).
  int gpioEncoderPinOne;
};
#endif