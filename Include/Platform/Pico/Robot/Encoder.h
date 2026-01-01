#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>

#include "QuadratureEncoder.h"
#include "Common/LogSystem.h"
#include "Platform/Pico/Config.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

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
     *
     * Stores current hardware count as offset for future readings
     */
    void reset();

    /**
     * @brief Returns current encoder tick count
     * @return Total ticks since last reset (positive or negative)
     */
    int getTickCount() const;

  private:
    /**
     * @brief Loads quadrature decoder PIO program into hardware (once per PIO instance)
     * @param pio_instance PIO hardware instance to load program into
     */
    void loadPIOProgram(PIO pio_instance);

    // PIO hardware resources
    const PIO pio_instance_;       // PIO block used by this encoder
    uint state_machine_;           // State machine index for PIO program

    // Encoder configuration
    int offset_ticks_;             // Offset for software reset functionality
    bool invert_direction_;        // Direction inversion flag
    int gpio_pin_;                 // Base GPIO pin number
};

#endif