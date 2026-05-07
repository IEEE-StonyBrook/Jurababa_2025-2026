#ifndef APP_LEDS_H
#define APP_LEDS_H

/**
 * @file leds.h
 * @brief Runtime status LED on the Waveshare RP2040-Zero NeoPixel (GPIO 16).
 *
 * Single PIO-driven WS2812 owned by this module. The boot flash and the
 * runtime competition-flow indicators all share one PIO state machine, so
 * they cannot run concurrently — but they don't need to: boot flash runs
 * before main, every other call is from the CLI thread between motion
 * sequences. The LED colour is the only operator-visible signal during a
 * fast run (no buzzer is wired), so the palette is chosen for distinctness
 * under fluorescent venue lighting rather than aesthetic reasons.
 */
namespace stage_led
{

// Boot indicator: flashes BOOT_LED_FLASH_COUNT times in green at
// BOOT_LED_BRIGHTNESS. Same behavior as the original main.cpp boot flash;
// this is its new home so runtime calls can share the PIO setup.
void flashBoot();

// Solid white — gesture detector is armed and waiting for an operator wave.
void setArmed();

// Solid colour keyed to the competition stage. 1=yellow (search), 2=cyan
// (return), 3=green (cardinal speed run), 5=magenta (diagonal speed run).
// Any other value is treated as setIdle().
void setStage(int stage);

// Solid bright cyan held for the goal-reached pause between stage 1 and
// stage 2. Visually distinct from setStage(2) so an observer can tell
// "still searching" from "at goal, about to return."
void setGoalReached();

// Three red flashes, blocking ~600 ms. Called from the abort path; the
// operator has already pressed BOOTSEL so spinning briefly is acceptable.
void flashAborted();

// Solid red — COMP is paused after an abort, waiting for the operator to
// press BOOTSEL a second time to resume. Distinct from flashAborted (the
// announcement) and from setArmed (white pulse, ready for gesture).
void setPaused();

// LED off.
void setIdle();

} // namespace stage_led

#endif
