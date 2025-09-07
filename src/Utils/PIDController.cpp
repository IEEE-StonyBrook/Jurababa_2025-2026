#include "PIDController.h"

PIDController::PIDController(float kP, float kI, float kD, float initialError,
                             float integralMaxValue, float deadbandToReturnZero)
    : kP(kP),
      kI(kI),
      kD(kD),
      previousCalculationError(initialError),
      integralMaxValue(integralMaxValue),
      deadbandToReturnZero(deadbandToReturnZero) {
  previousCalculationTime = get_absolute_time();
  accumulatedIntegral = 0.0f;
}

float PIDController::calculateOutput(float newError) {
  // Calculations for kP error.
  float proportionalOutput = kP * newError;

  if (kI == 0.0f and kD == 0.0f) {
    return proportionalOutput;
  }

  // Calculates dt in seconds, used for integral/derivative calculations.
  absolute_time_t timeNow = get_absolute_time();
  float timeDifferenceSec =
      absolute_time_diff_us(previousCalculationTime, timeNow) / 1e6f;
  previousCalculationTime = timeNow;

  // Calculations for kI error; clamps integral at max values.
  accumulatedIntegral += newError * timeDifferenceSec;
  accumulatedIntegral =
      fmax(fmin(accumulatedIntegral, integralMaxValue), -integralMaxValue);
  float integralOutput = kI * accumulatedIntegral;

  // Calculations for kD error.
  float derivativeOutput =
      kD * ((newError - previousCalculationError) / timeDifferenceSec);

  float output = proportionalOutput + integralOutput + derivativeOutput;

  // If output is less than deadband, return 0.
  return (fabs(output) < deadbandToReturnZero) ? 0.0f : output;
}

void PIDController::setInitialError(float initialError) {
  previousCalculationError = initialError;
}

void PIDController::setDeadband(float deadband) {
  deadbandToReturnZero = deadband;
}