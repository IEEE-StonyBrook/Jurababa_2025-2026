#include "drivers/encoder.h"

Encoder::Encoder(PIO pio_instance, int gpio_pin, bool invert_direction)
    : pio_instance_(pio_instance), gpio_pin_(gpio_pin), offset_ticks_(0),
      invert_direction_(invert_direction)
{
    state_machine_ = pio_claim_unused_sm(pio_instance_, true);
    load_pio_program(pio_instance_);
    quadrature_encoder_program_init(pio_instance_, state_machine_, gpio_pin_, 0);
}

void Encoder::load_pio_program(PIO pio_instance)
{
    static bool program_loaded = false;
    if (!program_loaded)
    {
        pio_add_program_at_offset(pio_instance, &quadrature_encoder_program, 0);
        program_loaded = true;
    }
}

int Encoder::ticks() const
{
    int raw_ticks           = quadrature_encoder_get_count(pio_instance_, state_machine_);
    int direction_corrected = raw_ticks * (invert_direction_ ? -1 : 1);
    return direction_corrected - offset_ticks_;
}

void Encoder::reset()
{
    LOG_DEBUG("Resetting encoder tick count to zero");
    offset_ticks_ += ticks();
}
