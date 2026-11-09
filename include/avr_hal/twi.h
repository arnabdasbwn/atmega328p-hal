/**
 * @file twi.h
 * @brief Bounded TWI controller transactions with error classification.
 * @details Uses A4/SDA and A5/SCL. External pull-ups and level translation belong in the circuit.
 * Transactions support a repeated START and last-byte NACK; arbitration loss releases the bus
 * without issuing STOP.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_TWI_H
#define AVR_HAL_TWI_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Configure TWI controller bus speed.
     * @param[in] frequency 100000 or 400000 Hz at the reference 16 MHz clock.
     * @retval AVR_OK Divider set and TWI enabled.
     * @retval AVR_ERROR_ARGUMENT Unsupported rate; registers unchanged.
     * @details Owns A4/SDA and A5/SCL. External pull-ups and compatible voltage translation are
     * circuit requirements. TWI interrupts disabled; global interrupt state unchanged. Call while
     * the bus is idle.
     */
    avr_status_t avr_twi_init(uint32_t frequency);
    /**
     * @brief Execute a write and optional repeated-START read.
     * @param[in] address Unshifted 7-bit peripheral address, 0x08..0x77.
     * @param[in] tx Caller-owned transmit buffer; may be NULL only for zero tx_length.
     * @param[in] tx_length Transmit byte count, 0..255; no buffer is retained.
     * @param[out] rx Receive buffer with rx_length bytes of capacity; NULL allowed only for zero
     * rx_length. May be partially filled on failure.
     * @param[in] rx_length Receive byte count, 0..255; zero/zero performs an address-write probe.
     * @param[in] budget Maximum unsuccessful polls for each protocol state, not for the whole
     * transaction.
     * @retval AVR_OK Transaction and STOP completed.
     * @retval AVR_ERROR_ARGUMENT Address or buffer invalid; bus unchanged.
     * @retval AVR_ERROR_IO TWI disabled or unexpected bus status.
     * @retval AVR_ERROR_NACK Address/data not acknowledged.
     * @retval AVR_ERROR_ARBITRATION Controller lost arbitration and released the bus without STOP.
     * @retval AVR_ERROR_TIMEOUT State or STOP timed out when no earlier transaction error exists.
     * @details Foreground only, one controller transaction at a time. No automatic retries. Reads
     * NACK the last byte. Attempts STOP after errors except arbitration loss. If STOP times out,
     * TWI is disabled; an earlier transaction error remains the returned status. Reinitialize
     * after diagnosing a stuck bus.
     */
    avr_status_t avr_twi_transfer(uint8_t address, const uint8_t *tx, uint8_t tx_length,
                                  uint8_t *rx, uint8_t rx_length, uint32_t budget);
#ifdef __cplusplus
}
#endif
#endif
