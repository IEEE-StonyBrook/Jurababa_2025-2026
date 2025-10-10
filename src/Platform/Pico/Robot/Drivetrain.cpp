#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"

Drivetrain::Drivetrain(Motor* leftMotor, Motor* rightMotor, Odometry* odometry,
                       ToF* leftToF, ToF* frontToF, ToF* rightToF)
    : leftMotor(leftMotor),
      rightMotor(rightMotor),
      odometry(odometry),
      leftToF(leftToF),
      frontToF(frontToF),
      rightToF(rightToF),
      leftIntegral(0.0f),
      leftPrevError(0.0f),
      leftDerivFilt(0.0f),
      rightIntegral(0.0f),
      rightPrevError(0.0f),
      rightDerivFilt(0.0f) {}

void Drivetrain::reset() {
  leftIntegral = rightIntegral = 0.0f;
  leftPrevError = rightPrevError = 0.0f;
  leftDerivFilt = rightDerivFilt = 0.0f;
  odometry->reset();
}

void Drivetrain::stop() {
  leftMotor->applyVoltage(0.0f);
  rightMotor->applyVoltage(0.0f);
}

void Drivetrain::runControl(float forwardVel, float angularVel,
                            float steeringCorrection) {
  // Update odometry
  odometry->update();

  // Effective angular velocity
  float effectiveRotVel = angularVel + steeringCorrection;

  // Convert robot velocity → wheel velocities
  float tangentSpeed =
      (effectiveRotVel * (M_PI / 180.0f)) * (WHEEL_BASE_MM / 2.0f);
  float leftTargetVel = forwardVel - tangentSpeed;
  float rightTargetVel = forwardVel + tangentSpeed;

  // Measured wheel velocities from odometry
  float leftMeasuredVel = odometry->getLeftWheelVelocityMMPerSec();
  float rightMeasuredVel = odometry->getRightWheelVelocityMMPerSec();

  // Feedforward
  float leftFF = feedforwardLeft(leftTargetVel, leftMeasuredVel);
  float rightFF = feedforwardRight(rightTargetVel, rightMeasuredVel);

  // PID corrections
  float leftPID = leftWheelPID(leftTargetVel, leftMeasuredVel);
  float rightPID = rightWheelPID(rightTargetVel, rightMeasuredVel);

  // Final motor voltages
  float leftCmd = leftFF + leftPID;
  float rightCmd = rightFF + rightPID;

  applyVoltage(leftMotor, leftCmd, "[Left]");
  applyVoltage(rightMotor, rightCmd, "[Right]");

  LOG_DEBUG(
      "[Drivetrain][RunControl] "
      "Fwd=" +
      std::to_string(forwardVel) + " Ang=" + std::to_string(angularVel) +
      " Ltarget=" + std::to_string(leftTargetVel) +
      " Rtarget=" + std::to_string(rightTargetVel) +
      " Lmeas=" + std::to_string(leftMeasuredVel) +
      " Rmeas=" + std::to_string(rightMeasuredVel) +
      " Lout=" + std::to_string(leftCmd) + " Rout=" + std::to_string(rightCmd));
}

// ---------------- PID ----------------
float Drivetrain::leftWheelPID(float targetVel, float measuredVel) {
  float error = targetVel - measuredVel;

  float P = LEFT_KP * error;
  leftIntegral += error / LOOP_FREQUENCY_HZ;
  float I = LEFT_KI * leftIntegral;

  float rawDeriv = (error - leftPrevError) * LOOP_FREQUENCY_HZ;
  leftDerivFilt = DERIV_ALPHA * leftDerivFilt + (1 - DERIV_ALPHA) * rawDeriv;
  float D = LEFT_KD * leftDerivFilt;

  leftPrevError = error;

  LOG_DEBUG("[Drivetrain][WheelPID][Left] Err=" + std::to_string(error) +
            " P=" + std::to_string(P) + " I=" + std::to_string(I) +
            " D=" + std::to_string(D));

  return P + I + D;
}

float Drivetrain::rightWheelPID(float targetVel, float measuredVel) {
  float error = targetVel - measuredVel;

  float P = RIGHT_KP * error;
  rightIntegral += error / LOOP_FREQUENCY_HZ;
  float I = RIGHT_KI * rightIntegral;

  float rawDeriv = (error - rightPrevError) * LOOP_FREQUENCY_HZ;
  rightDerivFilt = DERIV_ALPHA * rightDerivFilt + (1 - DERIV_ALPHA) * rawDeriv;
  float D = RIGHT_KD * rightDerivFilt;

  rightPrevError = error;

  LOG_DEBUG("[Drivetrain][WheelPID][Right] Err=" + std::to_string(error) +
            " P=" + std::to_string(P) + " I=" + std::to_string(I) +
            " D=" + std::to_string(D));

  return P + I + D;
}

// ---------------- Feedforward ----------------
float Drivetrain::feedforwardLeft(float targetVel, float lastVel) {
  float accel = (targetVel - lastVel) * LOOP_FREQUENCY_HZ;

  float kv = (targetVel >= 0) ? SPEED_FFL : SPEED_FBL;
  float ka = (targetVel >= 0) ? ACC_FFL : ACC_FBL;
  float bias = (targetVel >= 0) ? BIAS_FFL : BIAS_FBL;

  float ff = kv * targetVel + ka * accel + (targetVel != 0 ? bias : 0);

  LOG_DEBUG(
      "[Drivetrain][Feedforward][Left] TargetVel=" + std::to_string(targetVel) +
      " Accel=" + std::to_string(accel) + " Out=" + std::to_string(ff));

  return ff;
}

float Drivetrain::feedforwardRight(float targetVel, float lastVel) {
  float accel = (targetVel - lastVel) * LOOP_FREQUENCY_HZ;

  float kv = (targetVel >= 0) ? SPEED_FFR : SPEED_FBR;
  float ka = (targetVel >= 0) ? ACC_FFR : ACC_FBR;
  float bias = (targetVel >= 0) ? BIAS_FFR : BIAS_FBR;

  float ff = kv * targetVel + ka * accel + (targetVel != 0 ? bias : 0);

  LOG_DEBUG("[Drivetrain][Feedforward][Right] TargetVel=" +
            std::to_string(targetVel) + " Accel=" + std::to_string(accel) +
            " Out=" + std::to_string(ff));

  return ff;
}

// ---------------- Apply ----------------
void Drivetrain::applyVoltage(Motor* motor, float voltage,
                              const std::string& label) {
  if (fabs(voltage) < DEADBAND_MOTOR_VOLTS) {
    voltage = (voltage >= 0 ? DEADBAND_MOTOR_VOLTS : -DEADBAND_MOTOR_VOLTS);
  }
  motor->applyVoltage(voltage);

  LOG_DEBUG("[Drivetrain][ApplyVoltage]" + label +
            " Cmd=" + std::to_string(voltage) + " V");
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