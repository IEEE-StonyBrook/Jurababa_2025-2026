#include "../../../Include/Platform/Pico/Robot/Motor.h"

#define WHEEL_DIAMETER_MM 39.5f
#define TICKS_PER_WHEEL_REVOLUTION 1400.0f

#define PWM_WRAP 999u
#define LOOP_INTERVAL_MS 25.0f
#define MIN_DUTY_0_TO_1 0.225f

Motor::Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Encoder* encoder,
             Battery* battery, bool invertMotorDirection)
    : gpioMotorPinOne(gpioMotorPinOne),
      gpioMotorPinTwo(gpioMotorPinTwo),
      encoder(encoder),
      battery(battery),
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

  // Calculate velocity of motor wheel.
  float currPosMM = getWheelPositionMM();
  velMMPerSec = (currPosMM - lastPosMM) / dt_s;
  lastPosMM = currPosMM;

  // Calculate velocity error for correction.
  float velocityError = desiredVelMMPerSec - velMMPerSec;

  // Use PID to correct for velocity error.
  float pidPWM = pidVelocityController.calculateOutput(velocityError);

  // Use Feedforward to predict needed PWM.
  float ffPWM = 0.0f;
  // Takes into account direction (assymetric motion).
  if (desiredVelMMPerSec > 0.0f) {
    ffPWM = FF_KSF + FF_KVF * desiredVelMMPerSec;
  } else {
    ffPWM = -(FF_KSR + FF_KVR * std::fabs(desiredVelMMPerSec));
  }

  // Combine PID + Feedforward to calculate PWM. Add fallback for low PWM.
  float controlPWM = std::clamp(ffPWM + pidPWM, -1.0f, 1.0f);
  // LOG_DEBUG("Control PWM: " + std::to_string(controlPWM));
  if (fabs(controlPWM) < MIN_DUTY_0_TO_1 && desiredVelMMPerSec != 0.0f) {
    controlPWM = (controlPWM > 0 ? MIN_DUTY_0_TO_1 : -MIN_DUTY_0_TO_1);
  }

  // Send calculated PWM to motor.
  // applyPWM(controlPWM);
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

void Motor::configurePIDWithFF(float K_P, float K_I, float K_D, float FF_KVF,
                               float FF_KSF, float FF_KSR, float FF_KVR) {
  const float INIT_VEL_MM_PER_SEC = 0.0f;
  const float INTEGRAL_MAX_MM_PER_SEC = 1000.0f;
  const float DEADBAND_MM_PER_SEC = 0.1f;
  pidVelocityController =
      PIDController(K_P, K_I, K_D, INIT_VEL_MM_PER_SEC, INTEGRAL_MAX_MM_PER_SEC,
                    DEADBAND_MM_PER_SEC);

  this->FF_KVF = FF_KVF;
  this->FF_KSF = FF_KSF;
  this->FF_KSR = FF_KSR;
  this->FF_KVR = FF_KVR;
}

void Motor::setDesiredVelocityMMPerSec(float velMMPerSec) {
  desiredVelMMPerSec = velMMPerSec;
}

void Motor::stopMotor() {
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pidVelocityController.reset();
}