/**
 * @file system.h
 * @brief Analog comparator selection and idle sleep helpers.
 * @details Comparator setup owns D6/AIN0 and D7/AIN1 and disables their digital input buffers.
 * Sleep requires a configured wake interrupt. Event-driven sleeping additionally requires the
 * atomic flag/sleep pattern demonstrated by the lab.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_SYSTEM_H
#define AVR_HAL_SYSTEM_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Select AIN0/AIN1 for analog comparison.
     * @details Owns D6/AIN0 and D7/AIN1: inputs without pull-ups, digital input buffers disabled.
     * Clears ACME, comparator interrupt enable and pending ACI. Positive input is AIN0, negative
     * input AIN1. Nonblocking; global interrupt state unchanged.
     */
    void avr_comparator_init(void);
    /**
     * @brief Read the current comparator output bit.
     * @return True when ACO is set; normally AIN0 exceeds AIN1 after comparator propagation delay.
     * @details Call after comparator setup. Nonblocking; no register writes. This is a one-bit
     * comparison, not a voltage measurement and not a debounced threshold.
     */
    bool avr_comparator_output(void);
    /**
     * @brief Enter idle sleep once and return after wakeup.
     * @retval AVR_OK Sleep instruction executed and CPU resumed.
     * @retval AVR_ERROR_ARGUMENT Global interrupts were disabled; no sleep entered.
     * @details Foreground only. Caller must configure an enabled wake interrupt. Selects idle mode,
     * enables sleep and clears sleep-enable after waking. This helper alone does not atomically
     * test an event flag; use the race-free flag/sleep pattern in the lesson for event-driven
     * applications.
     */
    avr_status_t avr_sleep_idle(void);
#ifdef __cplusplus
}
#endif
#endif
