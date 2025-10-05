#include "../../../Include/Platform/Pico/Robot/Motor.h"

#include <algorithm>
#include <cmath>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define WHEEL_DIAMETER_MM 39.5f
#define TICKS_PER_WHEEL_REVOLUTION 1400.0f

#define PWM_WRAP 999u
#define LOOP_INTERVAL_MS 25.0f
#define MIN_DUTY_0_TO_1 0.4f

Motor::Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Encoder* encoder,
             bool invertMotorDirection)
    : gpioMotorPinOne(gpioMotorPinOne),
      gpioMotorPinTwo(gpioMotorPinTwo),
      encoder(encoder),
      invertMotorDirection(invertMotorDirection) {
  configureMotorPins();
  configureMotorPWM();

  lastTime = get_absolute_time();
  lastPosMM = getWheelPositionMM();
}

void Motor::configureMotorPins() {
  // Sets the GPIO to PWM function mode.
  gpio_set_function(gpioMotorPinOne, GPIO_FUNC_PWM);
  gpio_set_function(gpioMotorPinTwo, GPIO_FUNC_PWM);
}

void Motor::configureMotorPWM() {
  // Forward/backward channel for forward/backward movement respectively.
  pwmSliceNumber = pwm_gpio_to_slice_num(gpioMotorPinOne);
  fChannel = pwm_gpio_to_channel(gpioMotorPinOne);
  bChannel = pwm_gpio_to_channel(gpioMotorPinTwo);

  // Swaps forward/backward channels if motor is inverted.
  if (invertMotorDirection) {
    std::swap(fChannel, bChannel);
  }

  // Duty cycle of PWM range = [0, 999].
  pwm_config PWMConfig = pwm_get_default_config();
  pwm_init(pwmSliceNumber, &PWMConfig, false);
  pwm_set_wrap(pwmSliceNumber, PWM_WRAP);
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pwm_set_enabled(pwmSliceNumber, true);
}

float Motor::getWheelPositionMM() {
  // Converts ticks to revolutions to millimeters.
  float wheelPositionMM =
      (encoder->getCurrentEncoderTickCount() / TICKS_PER_WHEEL_REVOLUTION) *
      (static_cast<float>(M_PI) * WHEEL_DIAMETER_MM);

  // Negates if motor is inverted.
  return (invertMotorDirection ? -1.0f : 1.0f) * wheelPositionMM;
  ;
}

float Motor::getWheelVelocityMMPerSec() {
  // Velocity is updated in controlTick() periodically.
  return velMMPerSec;
}

void Motor::controlTick() {
  absolute_time_t now = get_absolute_time();
  int64_t dt_us = absolute_time_diff_us(lastTime, now);
  lastTime = now;
  float dt_s = static_cast<float>(dt_us) / 1e6f;
  dtAccum += dt_s;

  // Too little time -> accumulate dt until greater than loop frequency.
  int maxLoops = 5;
  const float LOOP_INTERVAL_S = LOOP_INTERVAL_MS / 1000.0f;
  while (dtAccum >= LOOP_INTERVAL_S && maxLoops-- > 0) {
    // Reset dt accumulator.
    dtAccum -= LOOP_INTERVAL_S;

    // Calculate velocity of motor wheel.
    float currPosMM = getWheelPositionMM();
    velMMPerSec = (currPosMM - lastPosMM) / LOOP_INTERVAL_S;
    lastPosMM = currPosMM;

    // Calculate velocity error for correction.
    float velocityError = desiredVelMMPerSec - velMMPerSec;

    // Use PID to correct for velocity error.
    float pidPWM = pidVelocityController.calculateOutput(velocityError);

    // Use Feedforward to predict needed PWM.
    float ffPWM = 0.0f;
    if (desiredVelMMPerSec != 0.0f) {
      // PWM to overcome static friction + velocity PWM prediction.
      ffPWM = FF_KS * (desiredVelMMPerSec > 0 ? 1.0f : -1.0f) +
              FF_KV * desiredVelMMPerSec;
      LOG_DEBUG("FF PWM: " + std::to_string(ffPWM));
      LOG_DEBUG("Desired Vel: " + std::to_string(desiredVelMMPerSec) +
                " | FF_KS: " + std::to_string(FF_KS) +
                " | FF_KV: " + std::to_string(FF_KV));
    }

    // Combine PID + Feedforward to calculate PWM. Add fallback for low PWM.
    float controlPWM = std::clamp(ffPWM + pidPWM, -1.0f, 1.0f);
    LOG_DEBUG("Control PWM: " + std::to_string(controlPWM));
    if (fabs(controlPWM) < MIN_DUTY_0_TO_1 && desiredVelMMPerSec != 0.0f) {
      controlPWM = (controlPWM > 0 ? MIN_DUTY_0_TO_1 : -MIN_DUTY_0_TO_1);
    }

    // LOG_DEBUG("Motor Tick | pos: " + std::to_string(currPosMM) +
    //           " mm | vel: " + std::to_string(velMMPerSec) +
    //           " mm/s | err: " + std::to_string(velocityError) + " | PID: " +
    //           std::to_string(pidPWM) + " | FF: " + std::to_string(ffPWM) +
    //           " | PWM: " + std::to_string(controlPWM));

    // Send calculated PWM to motor.
    applyPWM(controlPWM);
  }

  if (maxLoops <= 0) {
    // LOG_DEBUG("Motor Tick | Warning: skipped ticks due to backlog (dtAccum="
    // + std::to_string(dtAccum) + ")");
  }
}

void Motor::applyPWM(float duty) {
  // Max duty cycle is 100%, sign controls direction.
  duty = std::clamp(duty, -1.0f, 1.0f);
  bool forward = (duty >= 0.0f);

  // Scales duty to PWM's range.
  float pwmLevel = std::fabs(duty) * PWM_WRAP;
  // LOG_DEBUG("PWM Level1: " + std::to_string(pwmLevel));

  pwmLevel = std::clamp(pwmLevel, 0.0f, (float)PWM_WRAP);
  // LOG_DEBUG("PWM Level2: " + std::to_string(pwmLevel));

  // Sends the PWM signal to the channels.
  pwm_set_chan_level(pwmSliceNumber, forward ? fChannel : bChannel,
                     (uint16_t)pwmLevel);
  pwm_set_chan_level(pwmSliceNumber, forward ? bChannel : fChannel, 0);
}

void Motor::configurePIDWithFF(float K_P, float K_I, float K_D, float FF_KV,
                               float FF_KS) {
  const float INIT_VEL_MM_PER_SEC = 0.0f;
  const float INTEGRAL_MAX_MM_PER_SEC = 1000.0f;
  const float DEADBAND_MM_PER_SEC = 0.1f;
  pidVelocityController =
      PIDController(K_P, K_I, K_D, INIT_VEL_MM_PER_SEC, INTEGRAL_MAX_MM_PER_SEC,
                    DEADBAND_MM_PER_SEC);

  this->FF_KV = FF_KV;
  this->FF_KS = FF_KS;
}

void Motor::setDesiredVelocityMMPerSec(float velMMPerSec) {
  desiredVelMMPerSec = velMMPerSec;
}

void Motor::stopMotor() {
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pidVelocityController.reset();
}