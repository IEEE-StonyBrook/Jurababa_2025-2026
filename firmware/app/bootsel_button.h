#ifndef APP_BOOTSEL_BUTTON_H
#define APP_BOOTSEL_BUTTON_H

/**
 * @file bootsel_button.h
 * @brief Runtime read of the Waveshare RP2040-Zero BOOTSEL button.
 *
 * Used as the competition-mode panic-abort. BOOTSEL is wired to the QSPI
 * flash CS line; sampling it requires briefly overriding the pad output
 * driver and reading the SIO GPIO_HI_IN register. While that read is in
 * progress XIP cannot serve flash, so the worker function lives in SRAM
 * and disables interrupts for ~10 us per call.
 *
 * The public API is intentionally `abortRequested()` rather than
 * `bootselPressed()` so a future GPIO-based external panic button can OR
 * into the same source without churning the call sites.
 */
bool abortRequested();

#endif
