/**
 * @file register_device.h
 * @brief Reusable eight-bit-register TWI transport helpers.
 * @details Sends a register pointer before a repeated-START read, or sends a register/value write.
 * Device-specific initialization, scaling, identity checks and auto-increment rules belong to the
 * calling device driver.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_REGISTER_DEVICE_H
#define AVR_REGISTER_DEVICE_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Read sequential registers through TWI.
     * @param[in] address Unshifted peripheral address, 0x08..0x77.
     * @param[in] reg Eight-bit starting register address.
     * @param[out] data Buffer for length bytes; may be partly filled on bus failure. NULL valid
     * only at zero length.
     * @param[in] length Read length in bytes, 0..255.
     * @return Status from avr_twi_transfer, including NACK, arbitration or timeout.
     * @details Requires initialized TWI; foreground/exclusive bus use. Sends a register pointer
     * then a repeated START read, with AVR_POLL_BUDGET per state. Only appropriate for devices
     * whose protocol uses this transaction and supports auto-increment.
     */
    avr_status_t avr_device_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
    /**
     * @brief Write one eight-bit register through TWI.
     * @param[in] address Unshifted peripheral address, 0x08..0x77.
     * @param[in] reg Eight-bit register address.
     * @param[in] value Register data byte.
     * @return Status from avr_twi_transfer; a failure can follow partial bus activity.
     * @details Requires initialized TWI; foreground/exclusive bus use. Sends register address
     * followed by value with AVR_POLL_BUDGET per state; does not read back or retry.
     */
    avr_status_t avr_device_write(uint8_t address, uint8_t reg, uint8_t value);
#ifdef __cplusplus
}
#endif
#endif
