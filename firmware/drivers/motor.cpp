#include "drivers/motor.h"

Motor::Motor(int dir_pin, int pwm_pin, bool invert_direction)
    : dir_pin_(dir_pin), pwm_pin_(pwm_pin), invert_direction_(invert_direction)
{
    configurePins();
    configurePWM();
}

void Motor::configurePins()
{
    // Direction pin is a plain GPIO output
    gpio_init(dir_pin_);
    gpio_set_dir(dir_pin_, GPIO_OUT);
    gpio_put(dir_pin_, 0);

    // PWM pin uses hardware PWM
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
}

void Motor::configurePWM()
{
    pwm_slice_   = pwm_gpio_to_slice_num(pwm_pin_);
    pwm_channel_ = pwm_gpio_to_channel(pwm_pin_);

    pwm_config config = pwm_get_default_config();
    pwm_init(pwm_slice_, &config, false);
    pwm_set_wrap(pwm_slice_, PWM_WRAP);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
    pwm_set_enabled(pwm_slice_, true);
}

void Motor::applyDuty(float duty_cycle)
{
    duty_cycle = std::clamp(duty_cycle, -1.0f, 1.0f);

    // Set direction via CTRL pin (inverter makes IN1/IN2 pair)
    bool forward = (duty_cycle >= 0.0f);
    if (invert_direction_)
        forward = !forward;
    gpio_put(dir_pin_, forward ? 1 : 0);

    // Set speed via PWM (low PWM = short brake)
    float pwm_level = std::fabs(duty_cycle) * PWM_WRAP;
    pwm_level       = std::clamp(pwm_level, 0.0f, static_cast<float>(PWM_WRAP));
    pwm_set_chan_level(pwm_slice_, pwm_channel_, static_cast<uint16_t>(pwm_level));
}

void Motor::applyVoltage(float desired_volts, float battery_volts)
{
    if (battery_volts < 1.0f)
    {
        battery_volts = DEFAULT_BATTERY_VOLTAGE;
        LOG_DEBUG("Using default battery voltage");
    }
    applyDuty(desired_volts / battery_volts);
    LOG_DEBUG("Applying voltage of " + std::to_string(desired_volts) + " and duty of " +
              std::to_string((desired_volts / battery_volts)));
}

void Motor::stop()
{
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
}
