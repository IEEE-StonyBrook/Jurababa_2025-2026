#include "drivers/encoder.h"

#ifdef USE_MULTICORE_SENSORS
#include "app/multicore.h"
#endif

Encoder::Encoder(PIO pio_instance, int pin_a, int pin_b, bool invert_direction)
    : pio_instance_(pio_instance), pin_a_(pin_a), pin_b_(pin_b), offset_ticks_(0),
      invert_direction_(invert_direction)
{
    state_machine_ = pio_claim_unused_sm(pio_instance_, true);
    loadPIOProgram(pio_instance_);
    quadrature_encoder_program_init(pio_instance_, state_machine_, pin_a_, pin_b_, 0);
}

void Encoder::loadPIOProgram(PIO pio_instance)
{
    static bool program_loaded = false;
    if (!program_loaded)
    {
        pio_add_program(pio_instance, &quadrature_encoder_program);
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
    offset_ticks_ = quadrature_encoder_get_count(pio_instance_, state_machine_);
}
