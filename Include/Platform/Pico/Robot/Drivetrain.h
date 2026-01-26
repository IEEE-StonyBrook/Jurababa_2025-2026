/******************************************************************************
 * Project: Micromouse Robot
 * File: Drivetrain.h
 * -----
 * High-level drivetrain control. Uses odometry feedback and feedforward
 * prediction to calculate motor voltages from velocity and steering targets.
 ******************************************************************************/

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/Config.h"
#include "Motor.h"
#include "Odometry.h"
#include "IMU.h"
#include "ToF.h"

class Drivetrain {
 public:
  Drivetrain(Motor* leftMotor, Motor* rightMotor, Encoder* leftEncoder,
             Encoder* rightEncoder, IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF);

  // Reset odometry and all controller states.
  void reset();

  // Run control loop: update odometry and apply new motor voltages.
  void runControl(float forwardVelocityMMPerSec, float angularVelocityDegPerSec,
                  float steeringCorrection);

  // Immediately stop both motors.
  void stop();

  // Access odometry readings (distance, angle, velocity).
  Odometry* getOdometry();

  // Drive straight forward a set distance in mm at given velocity.
  void driveForwardMM(float distanceMM, float velocityMMPerSec = 300.0f);

  bool isWallLeft();
  bool isWallFront();
  bool isWallRight();
  
  ToF* leftToF;
  ToF* frontToF;
  ToF* rightToF;
  
 private:
  // Core controllers.
  float forwardPID();
  float rotationPID(float steeringCorrection);

  // Feedforward helpers.
  float feedforwardLeft(float wheelSpeed);
  float feedforwardRight(float wheelSpeed);

  Motor* leftMotor;
  Motor* rightMotor;
  Odometry odometry;

  

  // Targets.
  float targetForwardVel;  // mm/s
  float targetAngularVel;  // deg/s

  // Errors for PD control.
  float forwardError;
  float rotationError;
  float prevForwardError;
  float prevRotationError;
};

#endif  // DRIVETRAIN_H