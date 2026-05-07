#ifndef DRIVERS_BEACON_IR_H
#define DRIVERS_BEACON_IR_H

#include <cstdint>

/**
 * @brief Cheese Hunt IR beacon handshake on GP12/GP13.
 *
 * The beacon sends an active-low pulse when it is seen. The mouse measures the
 * pulse width in an IRQ, and a valid event causes a 40 kHz-ish IR burst that
 * deactivates the beacon. This driver is mutually exclusive with Bluetooth.
 */
class BeaconIr
{
  public:
    void begin();
    void reset();
    bool pollAndDeactivate();
    bool beaconOff() const { return beacon_off_; }

  private:
    static void gpioIrqThunk(unsigned int gpio, uint32_t events);
    void        handleIrq(unsigned int gpio, uint32_t events);
    void        sendBurst();

    static BeaconIr* instance_;

    volatile uint64_t pulse_start_us_   = 0;
    volatile bool     waiting_for_rise_ = false;
    volatile bool     beacon_detected_  = false;
    volatile bool     pulse_active_     = false;
    bool              beacon_off_       = false;
};

#endif // DRIVERS_BEACON_IR_H
