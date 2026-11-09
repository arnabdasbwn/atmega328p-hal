/**
 * @file gpio.h
 * @brief Uno R3 digital pin mapping and GPIO access.
 * @details Maps D0..D19 to ATmega328P ports without redefining vendor names. Input reads, output
 * latches and direction registers have different semantics. Atomic latch/direction updates restore
 * the previous interrupt state.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_GPIO_H
#define AVR_HAL_GPIO_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Pin direction and local pull-up policy. */
    typedef enum
    {
        AVR_GPIO_INPUT,        /**< High impedance with local pull-up disabled. */
        AVR_GPIO_INPUT_PULLUP, /**< Input with pull-up latch set; global PUD must be clear. */
        AVR_GPIO_OUTPUT        /**< Push-pull output, initialized LOW. */
    } avr_gpio_mode_t;
    /**
     * @brief Configure a Uno header pin direction and pull-up.
     * @param[in] pin Uno header pin 0..19; A0..A5 are 14..19.
     * @param[in] mode Input, input with pull-up, or output mode.
     * @retval AVR_OK Pin configuration was changed.
     * @retval AVR_ERROR_ARGUMENT Invalid pin or mode; registers unchanged.
     * @details Output mode starts with a LOW latch. Input mode disables its pull-up unless
     * explicitly requested. Atomic read-modify-write restores interrupt state. Does not change the
     * global PUD bit; pull-ups require PUD clear. Nonblocking.
     */
    avr_status_t avr_gpio_mode(uint8_t pin, avr_gpio_mode_t mode);
    /**
     * @brief Set one pin output latch.
     * @param[in] pin Uno header pin 0..19.
     * @param[in] high True sets the latch; false clears it.
     * @retval AVR_OK Latch updated.
     * @retval AVR_ERROR_ARGUMENT Pin invalid; registers unchanged.
     * @details Configure direction separately. On an input this enables/disables its pull-up,
     * subject to global PUD. Atomic read-modify-write restores interrupt state; nonblocking.
     */
    avr_status_t avr_gpio_write(uint8_t pin, bool high);
    /**
     * @brief Sample one pin electrical input.
     * @param[in] pin Uno header pin 0..19.
     * @param[out] high Non-null result pointer. Unchanged on argument error.
     * @retval AVR_OK The sampled input was stored.
     * @retval AVR_ERROR_ARGUMENT Invalid pin or null result.
     * @details Reads PINx without changing direction, latch or interrupt state. Nonblocking; this
     * is one sample, not a debounced level.
     */
    avr_status_t avr_gpio_read(uint8_t pin, bool *high);
    /**
     * @brief Toggle exactly one output latch.
     * @param[in] pin Uno header pin 0..19.
     * @retval AVR_OK Latch toggled.
     * @retval AVR_ERROR_ARGUMENT Pin invalid; registers unchanged.
     * @details Writes one to PINx, not a read-modify-write. The direction is unchanged. Toggling an
     * input changes its pull-up latch. Nonblocking.
     */
    avr_status_t avr_gpio_toggle(uint8_t pin);
#ifdef __cplusplus
}
#endif
#endif
