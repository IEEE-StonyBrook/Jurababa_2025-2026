#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"

#include <cmath>


const float RADIANS_PER_DEGREE = 2 * 3.14152790 / 360.0;
const float DEGREES_PER_RADIAN = 360.0 / 2 * 3.14152790;
const bool m_feedforward_enabled = true;
const bool m_controller_output_enabled = true;

Drivetrain::Drivetrain(Motor* leftMotor, Motor* rightMotor,
                       Encoder* leftEncoder, Encoder* rightEncoder, IMU* imu,
                       ToF* leftToF, ToF* frontToF, ToF* rightToF)
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
  float increment = targetForwardVel * LOOP_INTERVAL_S;
  forwardError += increment - odometry.getDeltaDistanceMM();
  float diff = forwardError - prevForwardError;
  prevForwardError = forwardError;
  float output = FWD_KP * forwardError + FWD_KD * diff;
  return output;
}

float Drivetrain::rotationPID(float steeringCorrection) {
  float increment = targetAngularVel * LOOP_INTERVAL_S;
  rotationError += increment - odometry.getDeltaAngleDeg();
  rotationError += steeringCorrection;
  float diff = rotationError - prevRotationError;
  prevRotationError = rotationError;
  float output = ROT_KP * rotationError + ROT_KD * diff;
  return output;
}

// Predict left motor voltage from wheel speed and acceleration.
float Drivetrain::feedforwardLeft(float wheelSpeed) {
  static float oldSpeed = wheelSpeed;
  float leftFF = wheelSpeed * SPEED_FFL;
  if (wheelSpeed > 0) {
    leftFF += BIAS_FFL;
  } else if (wheelSpeed < 0) {
    leftFF -= BIAS_FFL;
  } else {
    // No bias when the speed is 0
  }
  float acc = (wheelSpeed - oldSpeed) * LOOP_FREQUENCY_HZ;
  oldSpeed = wheelSpeed;
  float accFF = ACC_FFL * acc;
  leftFF += accFF;
  return leftFF;
}

// Predict right motor voltage from wheel speed and acceleration.
float Drivetrain::feedforwardRight(float wheelSpeed) {
  static float oldSpeed = wheelSpeed;
  float rightFF = wheelSpeed * SPEED_FFR;
  if (wheelSpeed > 0) {
    rightFF += BIAS_FFR;
  } else if (wheelSpeed < 0) {
    rightFF -= BIAS_FFR;
  } else {
    // No bias when the speed is 0
  }
  float acc = (wheelSpeed - oldSpeed) * LOOP_FREQUENCY_HZ;
  oldSpeed = wheelSpeed;
  float accFF = ACC_FFR * acc;
  rightFF += accFF;
  return rightFF;
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
  m_velocity = forwardVelocityMMPerSec;
  m_omega = angularVelocityDegPerSec;
  float pos_output = forwardPID();
  float rot_output = rotationPID(steeringCorrection);
  float left_output = 0;
  float right_output = 0;
  left_output = pos_output - rot_output;
  right_output = pos_output + rot_output;

  float tangent_speed = m_omega * WHEEL_BASE_MM / 2 * RADIANS_PER_DEGREE;
  float left_speed = m_velocity - tangent_speed;
  float right_speed = m_velocity + tangent_speed;
  float left_ff = feedforwardLeft(left_speed);
  float right_ff = feedforwardRight(right_speed);
  if (m_feedforward_enabled) {
    left_output += left_ff;
    right_output += right_ff;
  }
  if (m_controller_output_enabled) {
    leftMotor->applyVoltage(left_output);
    rightMotor->applyVoltage(right_output);
  }
}

void Drivetrain::stop() {
  leftMotor->stopMotor();
  rightMotor->stopMotor();
}

Odometry* Drivetrain::getOdometry() { return &odometry; }