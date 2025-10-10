#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"

Drivetrain::Drivetrain(Motor* leftMotor, Motor* rightMotor,
                       Encoder* leftEncoder, Encoder* rightEncoder, IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF)
    : leftMotor(leftMotor),
      rightMotor(rightMotor),
      odometry(leftEncoder, rightEncoder, imu),
      targetForwardVel(0.0f),
      targetAngularVel(0.0f),
      forwardError(0.0f),
      rotationError(0.0f),
      prevForwardError(0.0f),
      prevRotationError(0.0f) {}

void Drivetrain::reset() {
  LOG_DEBUG("Resetting odometry...");
  odometry.reset();
  LOG_DEBUG("Stopping motors...");
  stop();
  LOG_DEBUG("Clearing controller states...");
  forwardError = rotationError = 0.0f;
  prevForwardError = prevRotationError = 0.0f;
  targetForwardVel = targetAngularVel = 0.0f;
}

// Forward PD controller integrates target velocity into position error.
float Drivetrain::forwardPD() {
  float expectedChange = targetForwardVel * LOOP_INTERVAL_S;
  // Error between expected position change and actual change.
  // Proportional term from accumulated error.
  forwardError += expectedChange - odometry.getDeltaDistanceMM();
  // Derivative term from change in error.
  float errorRate = forwardError - prevForwardError;
  prevForwardError = forwardError;
  return (FWD_KP * forwardError) + (FWD_KD * errorRate);
}

// Rotation PD controller integrates angular velocity into rotation error.
float Drivetrain::rotationPD(float steeringCorrection) {
  float expectedChange = targetAngularVel * LOOP_INTERVAL_S;
  // Error between expected angle change and actual change.
  // Proportional term from accumulated error.
  rotationError += expectedChange - odometry.getDeltaAngleDeg();
  rotationError += steeringCorrection;  // Wall-following or trajectory adjust.
  // Derivative term from change in error.
  float errorRate = rotationError - prevRotationError;
  prevRotationError = rotationError;
  return (ROT_KP * rotationError) + (ROT_KD * errorRate);
}

// Predict left motor voltage from wheel speed and acceleration.
float Drivetrain::feedforwardLeft(float wheelSpeed) {
  static float lastSpeed = 0.0f;
  float voltage = 0.0f;

  if (wheelSpeed > 0) {
    // Forward: slope + static bias.
    voltage = (SPEED_FFL * wheelSpeed) + BIAS_FFL;
  } else if (wheelSpeed < 0) {
    // Reverse: slope + static bias.
    voltage = (SPEED_FBL * wheelSpeed) - BIAS_FBL;
  }

  // Acceleration contribution.
  float accel = (wheelSpeed - lastSpeed) * LOOP_FREQUENCY_HZ;
  lastSpeed = wheelSpeed;
  voltage += (ACC_FFL * accel);

  return voltage;
}

// Predict right motor voltage from wheel speed and acceleration.
float Drivetrain::feedforwardRight(float wheelSpeed) {
  static float lastSpeed = 0.0f;
  float voltage = 0.0f;

  if (wheelSpeed > 0) {
    // Forward.
    voltage = (SPEED_FFR * wheelSpeed) + BIAS_FFR;
  } else if (wheelSpeed < 0) {
    // Reverse.
    voltage = (SPEED_FBR * wheelSpeed) - BIAS_FBR;
  }

  // Acceleration contribution.
  float accel = (wheelSpeed - lastSpeed) * LOOP_FREQUENCY_HZ;
  lastSpeed = wheelSpeed;
  voltage += (ACC_FFR * accel);

  return voltage;
}

bool Drivetrain::isWallLeft() {
  return leftToF->getToFDistanceFromWallMM() < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Drivetrain::isWallFront() {
  return frontToF->getToFDistanceFromWallMM() < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool Drivetrain::isWallRight() {
  return rightToF->getToFDistanceFromWallMM() < TOF_RIGHT_WALL_THRESHOLD_MM;
}

// Main control loop: update odometry, compute voltages, and drive motors.
void Drivetrain::runControl(float forwardVelocityMMPerSec,
                            float angularVelocityDegPerSec,
                            float steeringCorrection) {
  targetForwardVel = forwardVelocityMMPerSec;
  targetAngularVel = angularVelocityDegPerSec;

  // LOG_DEBUG("Updating odometry...");
  odometry.update(); 

  // LOG_DEBUG("Calculating control outputs...");
  float forwardOut = forwardPD();
  float rotationOut = -rotationPD(steeringCorrection);

  // Combine forward and rotation control.
  float leftOut = forwardOut - rotationOut;
  float rightOut = forwardOut + rotationOut;

  // Calculate wheel speeds for feedforward terms.
  float tangentSpeed =
      (targetAngularVel * (M_PI / 180.0f)) * (WHEEL_BASE_MM / 2.0f);
  float leftWheelSpeed = targetForwardVel - tangentSpeed;
  float rightWheelSpeed = targetForwardVel + tangentSpeed;

  // Add feedforward predictions.
  leftOut += feedforwardLeft(leftWheelSpeed);
  rightOut += feedforwardRight(rightWheelSpeed);

  // LOG_DEBUG("Applying voltage of " + std::to_string(leftOut) + " V to left motor.");
  // LOG_DEBUG("Applying voltage of " + std::to_string(rightOut) + " V to right motor.");
  leftMotor->applyVoltage(leftOut);
  rightMotor->applyVoltage(rightOut);
}

void Drivetrain::stop() {
  leftMotor->stopMotor();
  rightMotor->stopMotor();
}

Odometry* Drivetrain::getOdometry() { return &odometry; }