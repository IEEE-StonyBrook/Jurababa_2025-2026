#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../../../Include/Common/LogSystem.h"
#include <cmath>
#include <string>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Drivetrain::Drivetrain(Motor* leftMotor, Motor* rightMotor,
                       Encoder* leftEncoder, Encoder* rightEncoder,
                       IMU* imu, ToF* leftToF, ToF* frontToF, ToF* rightToF)
    : leftMotor(leftMotor),
      rightMotor(rightMotor),
      odometry(leftEncoder, rightEncoder, imu),
      targetForwardVel(0.0f),
      targetAngularVel(0.0f),
      forwardError(0.0f),
      rotationError(0.0f),
      prevForwardError(0.0f),
      prevRotationError(0.0f),
      leftToF(leftToF),
      frontToF(frontToF),
      rightToF(rightToF) {
  LOG_DEBUG("[Drivetrain] Initialized.");
}

// -----------------------------------------------------------------------------
// Reset drivetrain state
// -----------------------------------------------------------------------------
void Drivetrain::reset() {
  LOG_DEBUG("[Drivetrain] Resetting...");
  odometry.reset();
  stop();
  forwardError = rotationError = 0.0f;
  prevForwardError = prevRotationError = 0.0f;
  targetForwardVel = targetAngularVel = 0.0f;
  LOG_DEBUG("[Drivetrain] Reset complete.");
}

// -----------------------------------------------------------------------------
// Forward PID
// -----------------------------------------------------------------------------
float Drivetrain::forwardPID() {
  float expectedDelta = targetForwardVel * LOOP_INTERVAL_S;
  float measuredDelta = odometry.getDeltaDistanceMM();
  forwardError += expectedDelta - measuredDelta;

  float rawDiff = forwardError - prevForwardError;
  prevForwardError = forwardError;

  static float derivFilt = 0.0f;
  const float alpha = 0.2f;
  derivFilt += alpha * (rawDiff - derivFilt);

  static float fwdInt = 0.0f;
  fwdInt += forwardError * LOOP_INTERVAL_S;

  const float INT_LIM = 200.0f;
  if (fwdInt > INT_LIM) fwdInt = INT_LIM;
  if (fwdInt < -INT_LIM) fwdInt = -INT_LIM;

  float output = (FWD_KP * forwardError) + (FWD_KI * fwdInt) + (FWD_KD * derivFilt);

  LOG_DEBUG("[Drivetrain][ForwardPID] Error=" + std::to_string(forwardError) +
            " Int=" + std::to_string(fwdInt) +
            " Deriv=" + std::to_string(derivFilt) +
            " Out=" + std::to_string(output));

  return output;
}

// -----------------------------------------------------------------------------
// Rotation PID
// -----------------------------------------------------------------------------
float Drivetrain::rotationPID(float steeringCorrection) {
  float expectedDelta = targetAngularVel * LOOP_INTERVAL_S;
  float measuredDelta = odometry.getDeltaAngleDeg();
  rotationError += expectedDelta - measuredDelta;
  rotationError += steeringCorrection;

  float rawDiff = rotationError - prevRotationError;
  prevRotationError = rotationError;

  static float derivFilt = 0.0f;
  const float alpha = 0.2f;
  derivFilt += alpha * (rawDiff - derivFilt);

  static float rotInt = 0.0f;
  rotInt += rotationError * LOOP_INTERVAL_S;

  const float INT_LIM = 30.0f;
  if (rotInt > INT_LIM) rotInt = INT_LIM;
  if (rotInt < -INT_LIM) rotInt = -INT_LIM;

  float output = (ROT_KP * rotationError) + (ROT_KI * rotInt) + (ROT_KD * derivFilt);

  LOG_DEBUG("[Drivetrain][RotationPID] Error=" + std::to_string(rotationError) +
            " Int=" + std::to_string(rotInt) +
            " Deriv=" + std::to_string(derivFilt) +
            " Out=" + std::to_string(output));

  return output;
}

// -----------------------------------------------------------------------------
// Feedforward helpers
// -----------------------------------------------------------------------------
float Drivetrain::feedforwardLeft(float wheelSpeed) {
  static float lastSpeed = 0.0f;
  float accel = (wheelSpeed - lastSpeed) * LOOP_FREQUENCY_HZ;
  lastSpeed = wheelSpeed;

  float voltage = 0.0f;
  if (wheelSpeed > 0) {
    voltage = (SPEED_FFL * wheelSpeed) + BIAS_FFL;
  } else if (wheelSpeed < 0) {
    voltage = (SPEED_FBL * wheelSpeed) - BIAS_FBL;
  }
  voltage += (ACC_FFL * accel);

  LOG_DEBUG("[Drivetrain][FeedforwardLeft] Speed=" + std::to_string(wheelSpeed) +
            " Accel=" + std::to_string(accel) +
            " V=" + std::to_string(voltage));
  return voltage;
}

float Drivetrain::feedforwardRight(float wheelSpeed) {
  static float lastSpeed = 0.0f;
  float accel = (wheelSpeed - lastSpeed) * LOOP_FREQUENCY_HZ;
  lastSpeed = wheelSpeed;

  float voltage = 0.0f;
  if (wheelSpeed > 0) {
    voltage = (SPEED_FFR * wheelSpeed) + BIAS_FFR;
  } else if (wheelSpeed < 0) {
    voltage = (SPEED_FBR * wheelSpeed) - BIAS_FBR;
  }
  voltage += (ACC_FFR * accel);

  LOG_DEBUG("[Drivetrain][FeedforwardRight] Speed=" + std::to_string(wheelSpeed) +
            " Accel=" + std::to_string(accel) +
            " V=" + std::to_string(voltage));
  return voltage;
}

// -----------------------------------------------------------------------------
// Wall detection
// -----------------------------------------------------------------------------
bool Drivetrain::isWallLeft() {
  bool detected = leftToF && (leftToF->getToFDistanceFromWallMM() < TOF_LEFT_WALL_THRESHOLD_MM);
  LOG_DEBUG("[Drivetrain][WallCheck] Left=" + std::string(detected ? "YES" : "NO"));
  return detected;
}

bool Drivetrain::isWallFront() {
  bool detected = frontToF && (frontToF->getToFDistanceFromWallMM() < TOF_FRONT_WALL_THRESHOLD_MM);
  LOG_DEBUG("[Drivetrain][WallCheck] Front=" + std::string(detected ? "YES" : "NO"));
  return detected;
}

bool Drivetrain::isWallRight() {
  bool detected = rightToF && (rightToF->getToFDistanceFromWallMM() < TOF_RIGHT_WALL_THRESHOLD_MM);
  LOG_DEBUG("[Drivetrain][WallCheck] Right=" + std::string(detected ? "YES" : "NO"));
  return detected;
}

// -----------------------------------------------------------------------------
// Main control loop
// -----------------------------------------------------------------------------
void Drivetrain::runControl(float forwardVelocityMMPerSec,
                            float angularVelocityDegPerSec,
                            float steeringCorrection) {
  targetForwardVel = forwardVelocityMMPerSec;
  targetAngularVel = angularVelocityDegPerSec;

  odometry.update();

  float forwardOut = forwardPID();
  float rotationOut = -rotationPID(steeringCorrection);

  float leftOut = forwardOut - rotationOut;
  float rightOut = forwardOut + rotationOut;

  float tangentSpeed = (targetAngularVel * (M_PI / 180.0f)) * (WHEEL_BASE_MM / 2.0f);
  float leftWheelSpeed = targetForwardVel - tangentSpeed;
  float rightWheelSpeed = targetForwardVel + tangentSpeed;

  LOG_DEBUG("[Drivetrain][RunControl] ForwardVel=" + std::to_string(targetForwardVel) +
            " AngularVel=" + std::to_string(targetAngularVel) +
            " ForwardOut=" + std::to_string(forwardOut) +
            " RotationOut=" + std::to_string(rotationOut) +
            " LeftOut=" + std::to_string(leftOut) +
            " RightOut=" + std::to_string(rightOut) +
            " Lwheel=" + std::to_string(leftWheelSpeed) +
            " Rwheel=" + std::to_string(rightWheelSpeed));

  leftOut += feedforwardLeft(leftWheelSpeed);
  rightOut += feedforwardRight(rightWheelSpeed);

  LOG_DEBUG("[Drivetrain][RunControl] Applying Left=" + std::to_string(leftOut) +
            " V Right=" + std::to_string(rightOut) + " V");

  leftMotor->applyVoltage(leftOut);
  rightMotor->applyVoltage(rightOut);
}

// -----------------------------------------------------------------------------
// Stop motors
// -----------------------------------------------------------------------------
void Drivetrain::stop() {
  LOG_DEBUG("[Drivetrain] Stopping motors.");
  leftMotor->stopMotor();
  rightMotor->stopMotor();
}

// -----------------------------------------------------------------------------
// Accessor for odometry
// -----------------------------------------------------------------------------
Odometry* Drivetrain::getOdometry() {
  return &odometry;
}