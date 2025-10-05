#include "../../../Include/Platform/Pico/Robot/Motor.h"

Motor::Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Battery* battery,
             bool invertDirection)
    : gpioMotorPinOne(gpioMotorPinOne),
      gpioMotorPinTwo(gpioMotorPinTwo),
      battery(battery),
      invertDirection(invertDirection) {
  configureMotorPins();
  configureMotorPWM();
}

void Motor::configureMotorPins() {
  // Sets the GPIOs to PWM function mode.
  gpio_set_function(gpioMotorPinOne, GPIO_FUNC_PWM);
  gpio_set_function(gpioMotorPinTwo, GPIO_FUNC_PWM);
}

void Motor::configureMotorPWM() {
  // Gets the slice and channels for the given GPIOs.
  pwmSliceNumber = pwm_gpio_to_slice_num(gpioMotorPinOne);
  forwardChannel = pwm_gpio_to_channel(gpioMotorPinOne);
  backwardChannel = pwm_gpio_to_channel(gpioMotorPinTwo);

  // Swaps forward/backward channels if motor direction is inverted.
  if (invertDirection) {
    std::swap(forwardChannel, backwardChannel);
  }

  // Initializes PWM configuration.
  pwm_config pwmConfig = pwm_get_default_config();
  pwm_init(pwmSliceNumber, &pwmConfig, false);
  pwm_set_wrap(pwmSliceNumber, PWM_WRAP);
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pwm_set_enabled(pwmSliceNumber, true);
}

void Motor::applyPWM(float dutyCycle) {
  // Clamps duty cycle to range [-1.0, 1.0].
  dutyCycle = std::clamp(dutyCycle, -1.0f, 1.0f);
  bool forward = (dutyCycle >= 0.0f);

  // Scales duty cycle to PWM resolution.
  float pwmLevel = std::fabs(dutyCycle) * PWM_WRAP;
  pwmLevel = std::clamp(pwmLevel, 0.0f, static_cast<float>(PWM_WRAP));

  // Sends PWM signal to the correct channel.
  pwm_set_chan_level(pwmSliceNumber, forward ? forwardChannel : backwardChannel,
                     static_cast<uint16_t>(pwmLevel));
  pwm_set_chan_level(pwmSliceNumber, forward ? backwardChannel : forwardChannel,
                     0);
}

void Motor::applyVoltage(float desiredVolts) {
  // Reads live battery voltage.
  // float batteryVolts = battery->readVoltage();
  float batteryVolts = DEFAULT_BATTERY_VOLTAGE;
  if (batteryVolts < 1.0f) return;  // Guard against invalid ADC read.

  // Converts desired volts to duty cycle ratio.
  float dutyCycle = desiredVolts / batteryVolts;
  applyPWM(dutyCycle);
}

void Motor::stopMotor() {
  // Stops the motor by setting both channels to zero.
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
}