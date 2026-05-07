#include "app/bootsel_button.h"

#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

namespace
{
// QSPI CS is the second pad in the QSPI bank (index 1). When XIP is running
// the QSPI controller drives this line; to read it as a button we briefly
// override OEOVER to high-impedance, sample SIO GPIO_HI_IN bit 1, then put
// the override back to NORMAL so XIP can resume.
constexpr uint kCsPinIndex = 1;

// Must live in SRAM: while OEOVER is overridden the flash cannot be read,
// so the function body itself cannot be fetched from XIP.
bool __no_inline_not_in_flash_func(read_bootsel)()
{
    const uint32_t flags = save_and_disable_interrupts();

    hw_write_masked(&ioqspi_hw->io[kCsPinIndex].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Allow the line to settle before sampling. Empirically ~5 us is enough;
    // pico-examples uses a tight delay loop here for the same reason.
    for (volatile int i = 0; i < 1000; ++i)
        ;

    const bool pressed = (sio_hw->gpio_hi_in & (1u << kCsPinIndex)) == 0;

    hw_write_masked(&ioqspi_hw->io[kCsPinIndex].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);
    return pressed;
}
} // namespace

bool abortRequested()
{
    return read_bootsel();
}
