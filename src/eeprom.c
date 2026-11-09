/**
 * @file eeprom.c
 * @brief Bounded internal EEPROM byte access.
 * @details Addresses 0..1023, with update-if-different writes. The foreground caller has exclusive
 * EEPROM ownership; timed write-enable sequences restore interrupt state. A byte operation does not
 * provide multi-byte power-fail atomicity.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/eeprom.h"
#include <avr/io.h>
#include <util/atomic.h>
/**
 * @brief Wait until EEPROM and Flash self-programming hardware are idle.
 * @param[in] budget Maximum unsuccessful busy polls.
 * @retval AVR_OK Both programming engines are idle.
 * @retval AVR_ERROR_TIMEOUT At least one programming engine remained active.
 * @details No registers are changed. Foreground-only exclusive EEPROM ownership is required.
 */
static avr_status_t ready(uint32_t budget)
{
    while (((EECR & _BV(EEPE)) != 0U) || ((SPMCSR & _BV(SPMEN)) != 0U))
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    return AVR_OK;
}
/** @see eeprom.h for the authoritative API contract. */
avr_status_t avr_eeprom_read(uint16_t address, uint8_t *value, uint32_t budget)
{
    if ((address > E2END) || (value == NULL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_status_t result = ready(budget);
    if (result != AVR_OK)
    {
        return result;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        EEAR = address;
        EECR |= _BV(EERE);
        *value = EEDR;
    }
    return AVR_OK;
}
/** @see eeprom.h for the authoritative API contract. */
avr_status_t avr_eeprom_update(uint16_t address, uint8_t value, uint32_t budget)
{
    uint8_t old;
    avr_status_t result = avr_eeprom_read(address, &old, budget);
    if ((result != AVR_OK) || (old == value))
    {
        return result;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        EEAR = address;
        EEDR = value;
        EECR &= (uint8_t)~(_BV(EEPM1) | _BV(EEPM0));
        EECR |= _BV(EEMPE);
        EECR |= _BV(EEPE);
    }
    return ready(budget);
}
