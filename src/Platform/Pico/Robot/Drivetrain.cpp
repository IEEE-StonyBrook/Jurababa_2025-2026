#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"

Drivetrain::Drivetrain(Motor* leftMotor, Motor* rightMotor, ToF* leftToF,
                       ToF* middleToF, ToF* rightToF, IMU* drivetrainIMU)
    : leftMotor(leftMotor),
      rightMotor(rightMotor),
      leftToF(leftToF),
      middleToF(middleToF),
      rightToF(rightToF),
      drivetrainIMU(drivetrainIMU) {}

bool Drivetrain::driveRobotForwardNumberOfCells(int numberOfCells) {
  const float CELL_LENGTH_MM = 167.0f;

  float cellsLengthMM = numberOfCells * CELL_LENGTH_MM;
  float leftMotorCurrentPosition = leftMotor->getMotorPositionMM();
  float rightMotorCurrentPosition = rightMotor->getMotorPositionMM();
  leftMotor->setContinuousDesiredMotorPositionMM(leftMotorCurrentPosition +
                                                 cellsLengthMM);
  rightMotor->setContinuousDesiredMotorPositionMM(rightMotorCurrentPosition +
                                                  cellsLengthMM);

  return true;
}

bool Drivetrain::turnRobotDegreesInPlace(int degreesToTurn) {
  const float K_P = 3.0f;
  const float MAX_ERROR_DEGREES = 0.1f;

  desiredDegreesYaw = drivetrainIMU->getNewYawAfterAddingDegrees(degreesToTurn);
  yawController = PIDController(K_P);
  yawController.setDeadband(MAX_ERROR_DEGREES);

  updateYawErrorSignsAndPIDOutput();
  leftMotor->setContinuousDesiredMotorVelocityMMPerSec(leftPIDOutputSign *
                                                       pidYawCalculatedOutput);
  rightMotor->setContinuousDesiredMotorVelocityMMPerSec(-leftPIDOutputSign *
                                                        pidYawCalculatedOutput);

  while (true) {
    updateYawErrorSignsAndPIDOutput();
    leftMotor->setDesiredVelocityMMPerSec(leftPIDOutputSign *
                                          pidYawCalculatedOutput);
    rightMotor->setDesiredVelocityMMPerSec(-leftPIDOutputSign *
                                           pidYawCalculatedOutput);
  }
}

void Drivetrain::updateYawErrorSignsAndPIDOutput() {
  yawError =
      desiredDegreesYaw - drivetrainIMU->getIMUYawDegreesNeg180ToPos180();
  pidYawCalculatedOutput = yawController.calculateOutput(yawError);

  // Assigns signs for left velocity based on which direction to turn.
  // Right is always the opposite for in-place turns.
  leftPIDOutputSign = (yawError > 0) ? 1 : -1;
}