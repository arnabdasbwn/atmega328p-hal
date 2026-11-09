/**
 * @file system.c
 * @brief Analog comparator selection and idle sleep helpers.
 * @details Comparator setup owns D6/AIN0 and D7/AIN1 and disables their digital input buffers.
 * Sleep requires a configured wake interrupt. Event-driven sleeping additionally requires the
 * atomic flag/sleep pattern demonstrated by the lab.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/system.h"
#include <avr/io.h>
#include <avr/sleep.h>
/** @see system.h for the authoritative API contract. */
void avr_comparator_init(void)
{
    ADCSRB &= (uint8_t)~_BV(ACME);
    DDRD &= (uint8_t)~(_BV(DDD6) | _BV(DDD7));
    PORTD &= (uint8_t)~(_BV(PORTD6) | _BV(PORTD7));
    DIDR1 |= _BV(AIN0D) | _BV(AIN1D);
    ACSR = _BV(ACI); /* Clear pending flag; leave interrupt disabled. */
}
/** @see system.h for the authoritative API contract. */
bool avr_comparator_output(void)
{
    return (ACSR & _BV(ACO)) != 0U;
}
/** @see system.h for the authoritative API contract. */
avr_status_t avr_sleep_idle(void)
{
    if ((SREG & _BV(SREG_I)) == 0U)
    {
        return AVR_ERROR_ARGUMENT;
    }
    SMCR = (uint8_t)(SMCR & (uint8_t)~(_BV(SM2) | _BV(SM1) | _BV(SM0)));
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    return AVR_OK;
}
