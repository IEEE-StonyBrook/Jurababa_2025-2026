#include "../../../Include/Platform/Pico/Robot/Motor.h"
#include "../../../Include/Common/LogSystem.h"
#include <string>

Motor::Motor(int gpioMotorPinOne, int gpioMotorPinTwo, Battery* battery,
             bool invertDirection)
    : gpioMotorPinOne(gpioMotorPinOne),
      gpioMotorPinTwo(gpioMotorPinTwo),
      battery(battery),
      invertDirection(invertDirection) {
  LOG_DEBUG("[Motor] Initializing...");
  configureMotorPins();
  configureMotorPWM();
  LOG_DEBUG("[Motor] Init complete (GPIO " + std::to_string(gpioMotorPinOne) +
            "," + std::to_string(gpioMotorPinTwo) +
            (invertDirection ? ", inverted)" : ", normal)"));
}

void Motor::configureMotorPins() {
  gpio_set_function(gpioMotorPinOne, GPIO_FUNC_PWM);
  gpio_set_function(gpioMotorPinTwo, GPIO_FUNC_PWM);
  LOG_DEBUG("[Motor] Configured motor pins as PWM.");
}

void Motor::configureMotorPWM() {
  pwmSliceNumber = pwm_gpio_to_slice_num(gpioMotorPinOne);
  forwardChannel = pwm_gpio_to_channel(gpioMotorPinOne);
  backwardChannel = pwm_gpio_to_channel(gpioMotorPinTwo);

  if (invertDirection) {
    std::swap(forwardChannel, backwardChannel);
    LOG_DEBUG("[Motor] Direction inverted (swapped channels).");
  }

  pwm_config pwmConfig = pwm_get_default_config();
  pwm_init(pwmSliceNumber, &pwmConfig, false);
  pwm_set_wrap(pwmSliceNumber, PWM_WRAP);
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  pwm_set_enabled(pwmSliceNumber, true);
  LOG_DEBUG("[Motor] PWM slice " + std::to_string(pwmSliceNumber) + " enabled.");
}

void Motor::applyPWM(float dutyCycle) {
  dutyCycle = std::clamp(dutyCycle, -1.0f, 1.0f);
  bool forward = (dutyCycle >= 0.0f);

  float pwmLevel = std::fabs(dutyCycle) * PWM_WRAP;
  pwmLevel = std::clamp(pwmLevel, 0.0f, static_cast<float>(PWM_WRAP));

  pwm_set_chan_level(pwmSliceNumber, forward ? forwardChannel : backwardChannel,
                     static_cast<uint16_t>(pwmLevel));
  pwm_set_chan_level(pwmSliceNumber, forward ? backwardChannel : forwardChannel,
                     0);

  // LOG_DEBUG("[Motor] applyPWM -> Duty=" + std::to_string(dutyCycle) +
  //           " (" + (forward ? "FWD" : "REV") +
  //           "), PWM=" + std::to_string(pwmLevel));
}

void Motor::applyVoltage(float desiredVolts) {
  float batteryVolts = DEFAULT_BATTERY_VOLTAGE;
  if (batteryVolts < 1.0f) {
    // LOG_ERROR("[Motor] Invalid battery voltage reading!");
    return;
  }

  float original = desiredVolts;

  // Deadband check
  if (std::fabs(desiredVolts) > 1e-3f && std::fabs(desiredVolts) < DEADBAND_MOTOR_VOLTS) {
    desiredVolts = (desiredVolts > 0 ? +DEADBAND_MOTOR_VOLTS : -DEADBAND_MOTOR_VOLTS);
    // LOG_DEBUG("[Motor] Deadband adjust: " + std::to_string(original) +
    //           " V -> " + std::to_string(desiredVolts) + " V");
  }

  desiredVolts = std::clamp(desiredVolts, -batteryVolts, batteryVolts);
  float dutyCycle = desiredVolts / batteryVolts;

  // LOG_DEBUG("[Motor] applyVoltage: " + std::to_string(original) +
  //           " V req -> " + std::to_string(desiredVolts) +
  //           " V clamped, Duty=" + std::to_string(dutyCycle));
  applyPWM(dutyCycle);
}

void Motor::stopMotor() {
  pwm_set_both_levels(pwmSliceNumber, 0, 0);
  LOG_DEBUG("[Motor] STOP command issued.");
}