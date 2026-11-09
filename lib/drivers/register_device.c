/**
 * @file register_device.c
 * @brief Reusable eight-bit-register TWI transport helpers.
 * @details Sends a register pointer before a repeated-START read, or sends a register/value write.
 * Device-specific initialization, scaling, identity checks and auto-increment rules belong to the
 * calling device driver.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "register_device.h"
#include "avr_hal/twi.h"
/** @see register_device.h for the authoritative API contract. */
avr_status_t avr_device_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length)
{
    return avr_twi_transfer(address, &reg, 1U, data, length, AVR_POLL_BUDGET);
}
/** @see register_device.h for the authoritative API contract. */
avr_status_t avr_device_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[2] = {reg, value};
    return avr_twi_transfer(address, bytes, 2U, NULL, 0U, AVR_POLL_BUDGET);
}
