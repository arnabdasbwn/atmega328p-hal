/**
 * @file gpio.c
 * @brief Uno R3 digital pin mapping and GPIO access.
 * @details Maps D0..D19 to ATmega328P ports without redefining vendor names. Input reads, output
 * latches and direction registers have different semantics. Atomic latch/direction updates restore
 * the previous interrupt state.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/gpio.h"
#include <avr/io.h>
#include <util/atomic.h>
/** @brief Resolved register pointers for one Uno header pin, internal to this module. */
typedef struct
{
    volatile uint8_t *ddr;   /**< Direction register; one selects output. */
    volatile uint8_t *port;  /**< Output latch / pull-up control register. */
    volatile uint8_t *input; /**< Electrical input / write-one-to-toggle register. */
    uint8_t mask;            /**< Exactly one bit selecting the resolved port pin. */
} gpio_registers_t;
/**
 * @brief Translate a Uno header pin into its register triplet and mask.
 * @param[in] pin Uno digital pin 0..19; 14..19 represent A0..A5.
 * @param[out] r Non-null internal result structure, unchanged for an invalid pin.
 * @return True when the pin exists on the supported board header.
 * @details Pure address selection; never accesses or modifies the register contents.
 */
static bool resolve(uint8_t pin, gpio_registers_t *r)
{
    uint8_t bit;
    if (pin < 8U)
    {
        r->ddr = &DDRD;
        r->port = &PORTD;
        r->input = &PIND;
        bit = pin;
    }
    else if (pin < 14U)
    {
        r->ddr = &DDRB;
        r->port = &PORTB;
        r->input = &PINB;
        bit = (uint8_t)(pin - 8U);
    }
    else if (pin < 20U)
    {
        r->ddr = &DDRC;
        r->port = &PORTC;
        r->input = &PINC;
        bit = (uint8_t)(pin - 14U);
    }
    else
    {
        return false;
    }
    r->mask = (uint8_t)(UINT8_C(1) << bit);
    return true;
}
/** @see gpio.h for the authoritative API contract. */
avr_status_t avr_gpio_mode(uint8_t pin, avr_gpio_mode_t mode)
{
    gpio_registers_t r;
    if (!resolve(pin, &r) || (mode > AVR_GPIO_OUTPUT) || (mode < AVR_GPIO_INPUT))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (mode == AVR_GPIO_OUTPUT)
        {
            *r.port &= (uint8_t)~r.mask;
            *r.ddr |= r.mask;
        }
        else
        {
            *r.ddr &= (uint8_t)~r.mask;
            if (mode == AVR_GPIO_INPUT_PULLUP)
            {
                *r.port |= r.mask;
            }
            else
            {
                *r.port &= (uint8_t)~r.mask;
            }
        }
    }
    return AVR_OK;
}
/** @see gpio.h for the authoritative API contract. */
avr_status_t avr_gpio_write(uint8_t pin, bool high)
{
    gpio_registers_t r;
    if (!resolve(pin, &r))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (high)
        {
            *r.port |= r.mask;
        }
        else
        {
            *r.port &= (uint8_t)~r.mask;
        }
    }
    return AVR_OK;
}
/** @see gpio.h for the authoritative API contract. */
avr_status_t avr_gpio_read(uint8_t pin, bool *high)
{
    gpio_registers_t r;
    if ((high == NULL) || !resolve(pin, &r))
    {
        return AVR_ERROR_ARGUMENT;
    }
    *high = (*r.input & r.mask) != 0U;
    return AVR_OK;
}
/** @see gpio.h for the authoritative API contract. */
avr_status_t avr_gpio_toggle(uint8_t pin)
{
    gpio_registers_t r;
    if (!resolve(pin, &r))
    {
        return AVR_ERROR_ARGUMENT;
    }
    *r.input = r.mask;
    return AVR_OK;
}
