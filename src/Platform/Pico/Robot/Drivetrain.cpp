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

float Drivetrain::forwardPID() {
    // Update accumulated forward error (Harrison-style)
    float expectedDelta = targetForwardVel * LOOP_INTERVAL_S;
    float measuredDelta = odometry.getDeltaDistanceMM();
    forwardError += expectedDelta - measuredDelta;

    // Derivative on error (with low-pass filter)
    float rawDiff = forwardError - prevForwardError;
    prevForwardError = forwardError;

    static float derivFilt = 0.0f;
    const float alpha = 0.2f;  // smoothing factor (0..1)
    derivFilt += alpha * (rawDiff - derivFilt);

    // Explicit integral term
    static float fwdInt = 0.0f;
    fwdInt += forwardError * LOOP_INTERVAL_S;

    // Anti-windup clamp
    const float INT_LIM = 200.0f;
    if (fwdInt > INT_LIM) fwdInt = INT_LIM;
    if (fwdInt < -INT_LIM) fwdInt = -INT_LIM;

    // PID output
    return (FWD_KP * forwardError) + (FWD_KI * fwdInt) + (FWD_KD * derivFilt);
}

float Drivetrain::rotationPID(float steeringCorrection) {
    // Update accumulated rotation error (Harrison-style)
    float expectedDelta = targetAngularVel * LOOP_INTERVAL_S;   // deg
    float measuredDelta = odometry.getDeltaAngleDeg();          // deg
    rotationError += expectedDelta - measuredDelta;

    LOG_DEBUG("expectedDelta: " + std::to_string(expectedDelta));
    LOG_DEBUG("measuredDelta: " + std::to_string(measuredDelta));

    // Add steering correction (treated like an extra angular error)
    rotationError += steeringCorrection;

    // Derivative on error (with low-pass filter)
    float rawDiff = rotationError - prevRotationError;
    prevRotationError = rotationError;

    static float derivFilt = 0.0f;
    const float alpha = 0.2f;
    derivFilt += alpha * (rawDiff - derivFilt);

    // Explicit integral term
    static float rotInt = 0.0f;
    rotInt += rotationError * LOOP_INTERVAL_S;

    // Anti-windup clamp
    const float INT_LIM = 30.0f;
    if (rotInt > INT_LIM) rotInt = INT_LIM;
    if (rotInt < -INT_LIM) rotInt = -INT_LIM;

    // PID output
    return (ROT_KP * rotationError) + (ROT_KI * rotInt) + (ROT_KD * derivFilt);
}

// float Drivetrain::rotationPID(float steeringCorrection) {
//     float actualRate = odometry.getDeltaAngleDeg() / LOOP_INTERVAL_S; // deg/sec
//     float error = targetAngularVel - actualRate;
//     error += steeringCorrection;
//     float errorRate = error - prevRotationError;
//     prevRotationError = error;
//     return (ROT_KP * error) + (ROT_KD * errorRate);
// }

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
  float forwardOut = forwardPID();
  float rotationOut = -rotationPID(steeringCorrection);

  // Combine forward and rotation control.
  float leftOut = forwardOut - rotationOut;
  float rightOut = forwardOut + rotationOut;
  LOG_DEBUG("Left PD Output: " + std::to_string(leftOut));
  LOG_DEBUG("Right PD Output: " + std::to_string(rightOut));
  LOG_DEBUG("Forward Output: " + std::to_string(forwardOut));
  LOG_DEBUG("Rotation Output: " + std::to_string(rotationOut));

  // Calculate wheel speeds for feedforward terms. 
  float tangentSpeed =
      (targetAngularVel * (M_PI / 180.0f)) * (WHEEL_BASE_MM / 2.0f);
  float leftWheelSpeed = targetForwardVel - tangentSpeed;
  float rightWheelSpeed = targetForwardVel + tangentSpeed; 
  LOG_DEBUG("Left Wheel Speed: " + std::to_string(leftWheelSpeed) + " mm/s");
  LOG_DEBUG("Right Wheel Speed: " + std::to_string(rightWheelSpeed) + " mm/s");

  // Add feedforward predictions.
  leftOut += -feedforwardLeft(leftWheelSpeed);
  rightOut += -feedforwardRight(rightWheelSpeed);
  LOG_DEBUG ("Left FF Output: " + std::to_string(feedforwardLeft(leftWheelSpeed)));
  LOG_DEBUG ("Right FF Output: " + std::to_string(feedforwardRight(rightWheelSpeed)));

  LOG_DEBUG("Applying voltage of " + std::to_string(leftOut) + " V to left motor.");
  LOG_DEBUG("Applying voltage of " + std::to_string(rightOut) + " V to right motor.");
  leftMotor->applyVoltage(leftOut);
  rightMotor->applyVoltage(rightOut);
}

void Drivetrain::stop() {
  leftMotor->stopMotor();
  rightMotor->stopMotor();
}

Odometry* Drivetrain::getOdometry() { return &odometry; }