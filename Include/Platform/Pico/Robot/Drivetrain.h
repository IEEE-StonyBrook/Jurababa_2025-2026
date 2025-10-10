#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "../../../Common/LogSystem.h"
#include "../Config.h"
#include "Motor.h"
#include "Odometry.h"
#include "ToF.h"


class Drivetrain {
 public:
  Drivetrain(Motor* leftMotor, Motor* rightMotor, Odometry* odometry,
             ToF* leftToF, ToF* frontToF, ToF* rightToF);

  void reset();
  void stop();

  // Main control loop
  void runControl(float forwardVel, float angularVel, float steeringCorrection);
  Odometry* getOdometry() { return odometry; }
  bool isWallLeft();
  bool isWallFront();
  bool isWallRight();

 private:
  // Hardware
  Motor* leftMotor;
  Motor* rightMotor;
  Odometry* odometry;

  // ToF Sensors
  ToF* leftToF;
  ToF* frontToF;
  ToF* rightToF;

  // Left PID state
  float leftIntegral;
  float leftPrevError;
  float leftDerivFilt;

  // Right PID state
  float rightIntegral;
  float rightPrevError;
  float rightDerivFilt;

  // Helpers
  float leftWheelPID(float targetVel, float measuredVel);
  float rightWheelPID(float targetVel, float measuredVel);
  float feedforwardLeft(float targetVel, float lastVel);
  float feedforwardRight(float targetVel, float lastVel);
  void applyVoltage(Motor* motor, float voltage, const std::string& label);
};

#endif