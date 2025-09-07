#include "./../../Include/Platform/Pico/Robot/Motor.h"

#include <cmath>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

Motor::Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Encoder* encoder,
             bool invertMotorDirection)
    : gpioMotorPinOne(gpioMotorPinOne),
      gpioMotorPinTwo(gpioMotorPinTwo),
      encoder(encoder),
      invertMotorDirection(invertMotorDirection) {
  setUpMotorPins();
  setUpMotorPWM();

  timeWhenVelocityWasLastCheckedMicroSec = get_absolute_time();
  positionWhenVelocityWasLastCheckedMM = getMotorPositionMM();
}

void Motor::setUpMotorPins() {
  // Sets the GPIO to PWM function mode.
  gpio_set_function(gpioMotorPinOne, GPIO_FUNC_PWM);
  gpio_set_function(gpioMotorPinTwo, GPIO_FUNC_PWM);
}

void Motor::setUpMotorPWM() {
  // Forward/backward channel for forward/backward movement respectively.
  pwmSliceNumber = pwm_gpio_to_slice_num(gpioMotorPinOne);
  forwardChannel = pwm_gpio_to_channel(gpioMotorPinOne);
  backwardChannel = pwm_gpio_to_channel(gpioMotorPinTwo);

  // Swaps forward/backward channels if motor is inverted.
  if (invertMotorDirection)
    forwardChannel, backwardChannel = backwardChannel, forwardChannel;

  // Duty cycle of PWM range = [0, 999].
  pwm_config PWMConfig = pwm_get_default_config();
  pwm_init(pwmSliceNumber, &PWMConfig, false);
  pwm_set_wrap(pwmSliceNumber, 999);
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pwm_set_enabled(pwmSliceNumber, true);
}

float Motor::getMotorPositionMM() {
  // Converts ticks --(rev/tick)-> revolutions --(mm/rev)-> millimeters.
  float motorPositionMM =
      (encoder->getCurrentEncoderTickCount() / TICKS_PER_WHEEL_REVOLUTION) *
      (3.1415 * WHEEL_DIAMETER_MM);

  // Negates if motor is inverted.
  motorPositionMM = (invertMotorDirection ? -1 : 1) * motorPositionMM;

  return motorPositionMM;
}

float Motor::getMotorVelocityMMPerSec() {
  absolute_time_t timeNow = get_absolute_time();

  // Velocity = millimeter difference / second difference.
  float velocityMMPerSec =
      (getMotorPositionMM() - positionWhenVelocityWasLastCheckedMM) /
      (absolute_time_diff_us(timeWhenVelocityWasLastCheckedMicroSec, timeNow) /
       1e6f);
  timeWhenVelocityWasLastCheckedMicroSec = timeNow;

  return velocityMMPerSec;
}

void Motor::setContinuousDesiredMotorPositionMM(float desiredPositionMM) {
  const float K_P = 1.0f;
  const float MAX_MOTOR_POSITION_ERROR = 3.0f;

  pidPositionController = PIDController(K_P);
  updatePositionErrorAndPIDOutput();
  setContinuousDesiredMotorVelocityMMPerSec(pidPositionCalculatedOutput);

  while (true) {
    updatePositionErrorAndPIDOutput();
    desiredVelocityMMPerSec = pidPositionCalculatedOutput;
  }
}

void Motor::updatePositionErrorAndPIDOutput() {
  motorPositionErrorMM = desiredPositionMM - getMotorPositionMM();
  pidPositionCalculatedOutput =
      pidPositionController.calculateOutput(motorPositionErrorMM);
}

void Motor::setContinuousDesiredMotorVelocityMMPerSec(
    float desiredVelMMPerSec) {
  const float MAX_MOTOR_VELOCITY_ERROR_MM_PER_SEC = 1.0f;
  const float MAX_MOTOR_VELOCITY_MM_PER_SEC = 500.0f;
  const int SLEEP_BETWEEN_PID_CYCLES_MS = 20;
  const float MIN_MOTOR_VELOCITY_MM_PER_SEC = 1.0f;

  desiredVelocityMMPerSec = desiredVelMMPerSec;

  if (desiredVelocityMMPerSec < MIN_MOTOR_VELOCITY_MM_PER_SEC) {
    stopMotor();
    return;
  }

  float feedforwardOutput =
      feedforwardkS + feedforwardkV * desiredVelocityMMPerSec;

  // Stops sending new PWM values when the actual RPM is near desired RPM.
  while (true) {
    updateVelocityErrorAndPIDOutput();
    if (fabs(motorVelocityErrorMMPerSec) >
        MAX_MOTOR_VELOCITY_ERROR_MM_PER_SEC) {
      return;
    }

    float calculatedVelocityMMPerSec =
        feedforwardOutput + pidVelocityCalculatedOutput;
    setMotorPWMPercentageNeg1ToPos1(calculatedVelocityMMPerSec /
                                    MAX_MOTOR_VELOCITY_MM_PER_SEC);
    sleep_ms(SLEEP_BETWEEN_PID_CYCLES_MS);
  };
}

void Motor::updateVelocityErrorAndPIDOutput() {
  motorVelocityErrorMMPerSec =
      desiredVelocityMMPerSec - getMotorVelocityMMPerSec();
  pidVelocityCalculatedOutput =
      pidVelocityController.calculateOutput(motorVelocityErrorMMPerSec);
}

void Motor::setUpPIDControllerWithFeedforward(float kP, float kI, float kD,
                                              float feedforwardkS,
                                              float feedforwardkV) {
  const float INITIAL_VELOCITY_MM_PER_SEC = 0.0f;
  const float INTEGRAL_MAX_VALUE_MM_PER_SEC = 1000.0f;
  const float DEADBAND_TO_RETURN_ZERO_MM_PER_SEC = 0.1f;
  pidVelocityController = PIDController(kP, kI, kD, INITIAL_VELOCITY_MM_PER_SEC,
                                        INTEGRAL_MAX_VALUE_MM_PER_SEC,
                                        DEADBAND_TO_RETURN_ZERO_MM_PER_SEC);

  this->feedforwardkS = feedforwardkS;
  this->feedforwardkV = feedforwardkV;
}

void Motor::setMotorPWMPercentageNeg1ToPos1(float PWMPercentage) {
  isMovingForward = !std::signbit(PWMPercentage);

  // Closes PWM output to [0, 999]
  float PWMOutput = fabs(PWMPercentage * 999.0f);
  PWMOutput = fmax(fmin(PWMOutput, 999.0f), 0.0f);

  // Sends PWM output to forward/reverse channel.
  pwm_set_chan_level(pwmSliceNumber,
                     isMovingForward ? forwardChannel : backwardChannel,
                     (uint16_t)PWMOutput);
  pwm_set_chan_level(pwmSliceNumber,
                     isMovingForward ? backwardChannel : forwardChannel, 0);
}

void Motor::setDesiredVelocityMMPerSec(float desiredVelMMPerSec) {
  desiredVelocityMMPerSec = desiredVelMMPerSec;
}

void Motor::stopMotor() { pwm_set_both_levels(pwmSliceNumber, 0, 0); }