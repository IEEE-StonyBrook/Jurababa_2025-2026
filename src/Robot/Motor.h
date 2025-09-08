#ifndef MOTOR_H
#define MOTOR_H

#include "Encoder.h"
#include "../Utils/PIDController.h"

class Motor {
 public:
  Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Encoder* encoder,
        bool invertMotorDirection = false);
  ~Motor();

  float getMotorPositionMM();
  float getMotorVelocityMMPerSec();
  void setContinuousDesiredMotorVelocityMMPerSec(float desiredVelocityMMPerSec);
  void setUpPIDControllerWithFeedforward(float kP, float kI, float kD,
                                         float feedforwardkS = 0.0f,
                                         float feedforwardkV = 0.0f);
  void setContinuousDesiredMotorPositionMM(float desiredPositionMM);
  void setDesiredVelocityMMPerSec(float desiredVelMMPerSec);

  const float WHEEL_DIAMETER_MM = 31.0f;
  const float TICKS_PER_WHEEL_REVOLUTION = 1400.0f;

 private:
  void setUpMotorPins();
  void setUpMotorPWM();
  void setMotorPWMPercentageNeg1ToPos1(float PWMPercentage);

  Encoder* encoder;
  int gpioMotorPinOne, gpioMotorPinTwo;
  bool invertMotorDirection;

  int pwmSliceNumber;
  int forwardChannel, backwardChannel;
  bool isMovingForward;

  absolute_time_t timeWhenVelocityWasLastCheckedMicroSec;
  float positionWhenVelocityWasLastCheckedMM;

  PIDController pidVelocityController, pidPositionController;
  float desiredPositionMM, desiredVelocityMMPerSec;
  float feedforwardkS, feedforwardkV;
  float motorPositionErrorMM, motorVelocityErrorMMPerSec,
      pidPositionCalculatedOutput, pidVelocityCalculatedOutput;
  void updatePositionErrorAndPIDOutput();
  void updateVelocityErrorAndPIDOutput();
  void stopMotor();
};

#endif