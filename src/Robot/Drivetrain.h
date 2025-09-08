#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "IMU.h"
#include "Motor.h"
#include "ToF.h"

class Drivetrain {
 public:
  Drivetrain(Motor* leftMotor, Motor* rightMotor, ToF* leftToF, ToF* middleToF,
             ToF* rightToF, IMU* drivetrainIMU);
  ~Drivetrain();

  bool driveRobotForwardNumberOfCells(int numberOfCells);
  bool turnRobotDegreesInPlace(int degreesToTurn);

 private:
  Motor* leftMotor;
  Motor* rightMotor;
  ToF* leftToF;
  ToF* middleToF;
  ToF* rightToF;
  IMU* drivetrainIMU;

  PIDController yawController;
  float desiredDegreesYaw;
  float yawError;
  float pidYawCalculatedOutput;
  float leftPIDOutputSign;

  void updateYawErrorSignsAndPIDOutput();
};
#endif