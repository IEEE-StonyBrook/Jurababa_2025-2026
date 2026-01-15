#include "drivers/motor.h"

Motor::Motor(int gpio_pin_one, int gpio_pin_two, bool invert_direction)
    : gpio_pin_one_(gpio_pin_one), gpio_pin_two_(gpio_pin_two), invert_direction_(invert_direction)
{
    configurePins();
    configurePWM();
}

void Motor::configurePins()
{
    gpio_set_function(gpio_pin_one_, GPIO_FUNC_PWM);
    gpio_set_function(gpio_pin_two_, GPIO_FUNC_PWM);
}

void Motor::configurePWM()
{
    pwm_slice_number_ = pwm_gpio_to_slice_num(gpio_pin_one_);
    forward_channel_  = pwm_gpio_to_channel(gpio_pin_one_);
    backward_channel_ = pwm_gpio_to_channel(gpio_pin_two_);

    if (invert_direction_)
    {
        std::swap(forward_channel_, backward_channel_);
    }

    pwm_config config = pwm_get_default_config();
    pwm_init(pwm_slice_number_, &config, false);
    pwm_set_wrap(pwm_slice_number_, PWM_WRAP);
    pwm_set_both_levels(pwm_slice_number_, 0, 0);
    pwm_set_enabled(pwm_slice_number_, true);
}

void Motor::applyDuty(float duty_cycle)
{
    duty_cycle      = std::clamp(duty_cycle, -1.0f, 1.0f);
    bool is_forward = (duty_cycle >= 0.0f);

    float pwm_level = std::fabs(duty_cycle) * PWM_WRAP;
    pwm_level       = std::clamp(pwm_level, 0.0f, static_cast<float>(PWM_WRAP));

    int active_channel   = is_forward ? forward_channel_ : backward_channel_;
    int inactive_channel = is_forward ? backward_channel_ : forward_channel_;

    pwm_set_chan_level(pwm_slice_number_, active_channel, static_cast<uint16_t>(pwm_level));
    pwm_set_chan_level(pwm_slice_number_, inactive_channel, 0);
}

void Motor::applyVoltage(float desired_volts, float battery_volts)
{
    if (battery_volts < 1.0f)
    {
        battery_volts = DEFAULT_BATTERY_VOLTAGE;
    }
    applyDuty(desired_volts / battery_volts);
}

void Motor::stop()
{
    pwm_set_both_levels(pwm_slice_number_, 0, 0);
}
