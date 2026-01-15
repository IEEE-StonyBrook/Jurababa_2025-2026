#ifndef DRIVERS_ENCODER_H
#define DRIVERS_ENCODER_H

#include <cstdint>

#include "common/log.h"
#include "config/config.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "quadrature_encoder/QuadratureEncoder.h"

/**
 * @brief Quadrature encoder reader using PIO hardware
 *
 * Implements a hardware-accelerated quadrature encoder decoder using the
 * Raspberry Pi Pico's PIO (Programmable I/O) peripheral. Tracks motor
 * rotation with high accuracy and low CPU overhead.
 */
class Encoder
{
  public:
    /**
     * @brief Constructs encoder reader with PIO configuration
     * @param pio_instance PIO hardware instance (pio0 or pio1)
     * @param gpio_pin Base GPIO pin (channel B assumed to be next sequential pin)
     * @param invert_direction If true, inverts tick count direction
     */
    Encoder(PIO pio_instance, int gpio_pin, bool invert_direction = false);

    /**
     * @brief Resets encoder tick count to zero
     */
    void reset();

    /**
     * @brief Returns current encoder tick count
     * @return Total ticks since last reset (positive or negative)
     */
    int ticks() const;

  private:
    void loadPIOProgram(PIO pio_instance);

    const PIO pio_instance_;
    uint      state_machine_;
    int       offset_ticks_;
    bool      invert_direction_;
    int       gpio_pin_;
};

#endif
