/**
 * @file eeprom.h
 * @brief Bounded internal EEPROM byte access.
 * @details Addresses 0..1023, with update-if-different writes. The foreground caller has exclusive
 * EEPROM ownership; timed write-enable sequences restore interrupt state. A byte operation does not
 * provide multi-byte power-fail atomicity.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_EEPROM_H
#define AVR_HAL_EEPROM_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Read one byte from the internal EEPROM.
     * @param[in] address EEPROM byte address, 0..1023.
     * @param[out] value Non-null destination; unchanged on errors.
     * @param[in] budget Unsuccessful ready polls allowed while EEPROM/Flash programming is active.
     * @retval AVR_OK Byte stored in value.
     * @retval AVR_ERROR_ARGUMENT Invalid address or null output.
     * @retval AVR_ERROR_TIMEOUT Programming hardware remained busy.
     * @details Foreground only with exclusive EEPROM ownership. Protects the address/read sequence
     * using a restore-state critical section. Does not write EEPROM or change global interrupt
     * policy.
     */
    avr_status_t avr_eeprom_read(uint16_t address, uint8_t *value, uint32_t budget);
    /**
     * @brief Write an EEPROM byte only if it differs.
     * @param[in] address EEPROM byte address, 0..1023.
     * @param[in] value Replacement byte.
     * @param[in] budget Poll bound for each readiness wait; not a millisecond deadline.
     * @retval AVR_OK Byte already matched or write completed.
     * @retval AVR_ERROR_ARGUMENT Address out of range.
     * @retval AVR_ERROR_TIMEOUT Hardware remained busy; a started write may still complete later.
     * @details Foreground/exclusive ownership only. Uses erase-and-write mode and the protected
     * EEMPE/EEPE sequence. Restores interrupt state after starting the write, then polls
     * completion. Not a power-fail-safe multi-byte transaction.
     */
    avr_status_t avr_eeprom_update(uint16_t address, uint8_t value, uint32_t budget);
#ifdef __cplusplus
}
#endif
#endif
