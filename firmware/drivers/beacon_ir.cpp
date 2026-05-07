#include "drivers/beacon_ir.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "common/log.h"
#include "config/config.h"

BeaconIr* BeaconIr::instance_ = nullptr;

void BeaconIr::begin()
{
    instance_ = this;

    gpio_init(PIN_IR_EMITTER);
    gpio_set_dir(PIN_IR_EMITTER, GPIO_OUT);
    gpio_put(PIN_IR_EMITTER, 0);

    gpio_init(PIN_IR_RECEIVER);
    gpio_set_dir(PIN_IR_RECEIVER, GPIO_IN);
    gpio_pull_up(PIN_IR_RECEIVER);

    reset();
    gpio_set_irq_enabled_with_callback(PIN_IR_RECEIVER, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                       true, &BeaconIr::gpioIrqThunk);
    LOG_INFO("Beacon IR ready on emitter GP" << PIN_IR_EMITTER << " receiver GP"
                                             << PIN_IR_RECEIVER);
}

void BeaconIr::reset()
{
    const uint32_t irq_state = save_and_disable_interrupts();
    pulse_start_us_          = 0;
    waiting_for_rise_        = false;
    beacon_detected_         = false;
    pulse_active_            = false;
    beacon_off_              = false;
    restore_interrupts(irq_state);
    gpio_put(PIN_IR_EMITTER, 0);
}

bool BeaconIr::pollAndDeactivate()
{
    bool detected = false;
    {
        const uint32_t irq_state = save_and_disable_interrupts();
        detected                 = beacon_detected_;
        beacon_detected_         = false;
        restore_interrupts(irq_state);
    }

    if (!detected || beacon_off_)
        return false;

    LOG_INFO("Cheese Hunt: beacon detected, sending deactivation burst");
    sendBurst();
    beacon_off_ = true;
    sleep_ms(BEACON_IR_POST_DETECT_LOG_MS);
    return true;
}

void BeaconIr::gpioIrqThunk(unsigned int gpio, uint32_t events)
{
    if (instance_ != nullptr)
        instance_->handleIrq(gpio, events);
}

void BeaconIr::handleIrq(unsigned int gpio, uint32_t events)
{
    if (gpio != PIN_IR_RECEIVER)
        return;

    const uint64_t now_us = time_us_64();
    if ((events & GPIO_IRQ_EDGE_FALL) != 0)
    {
        pulse_start_us_   = now_us;
        waiting_for_rise_ = true;
    }

    if ((events & GPIO_IRQ_EDGE_RISE) != 0 && waiting_for_rise_)
    {
        const uint64_t pulse_time_us = now_us - pulse_start_us_;
        waiting_for_rise_            = false;

        if (!pulse_active_ && pulse_time_us >= BEACON_IR_MIN_PULSE_US &&
            pulse_time_us <= BEACON_IR_MAX_PULSE_US)
        {
            beacon_detected_ = true;
        }
    }
}

void BeaconIr::sendBurst()
{
    pulse_active_ = true;
    for (int i = 0; i < BEACON_IR_BURST_CYCLES; ++i)
    {
        gpio_put(PIN_IR_EMITTER, 1);
        sleep_us(BEACON_IR_BURST_HIGH_US);
        gpio_put(PIN_IR_EMITTER, 0);
        sleep_us(BEACON_IR_BURST_LOW_US);
    }
    pulse_active_ = false;
}
