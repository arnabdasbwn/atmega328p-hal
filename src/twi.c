/**
 * @file twi.c
 * @brief Bounded TWI controller transactions with error classification.
 * @details Uses A4/SDA and A5/SCL. External pull-ups and level translation belong in the circuit.
 * Transactions support a repeated START and last-byte NACK; arbitration loss releases the bus
 * without issuing STOP.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/twi.h"
#include <avr/io.h>
#include <util/twi.h>
/**
 * @brief Wait for TWINT and classify the completed bus state.
 * @param[in] expected Masked TWSR state required by the current protocol step.
 * @param[in] budget Maximum unsuccessful TWINT polls, in iterations.
 * @return AVR_OK, TIMEOUT, ARBITRATION, NACK or IO status for that step.
 * @details Reads status only; the transaction owner is responsible for STOP/recovery.
 */
static avr_status_t wait_status(uint8_t expected, uint32_t budget)
{
    while ((TWCR & _BV(TWINT)) == 0U)
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    uint8_t status = (uint8_t)(TWSR & 0xF8U);
    if (status == expected)
    {
        return AVR_OK;
    }
    if (status == TW_MT_ARB_LOST)
    {
        return AVR_ERROR_ARBITRATION;
    }
    if ((status == TW_MT_SLA_NACK) || (status == TW_MT_DATA_NACK) || (status == TW_MR_SLA_NACK))
    {
        return AVR_ERROR_NACK;
    }
    return AVR_ERROR_IO;
}
/**
 * @brief Request STOP and wait for the hardware to release its request bit.
 * @param[in] budget Maximum unsuccessful TWSTO polls.
 * @retval AVR_OK STOP completed.
 * @retval AVR_ERROR_TIMEOUT STOP remained pending; TWI was disabled for recovery.
 * @details Must not be used after losing bus arbitration to another controller.
 */
static avr_status_t stop(uint32_t budget)
{
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    while ((TWCR & _BV(TWSTO)) != 0U)
    {
        if (budget == 0UL)
        {
            TWCR = 0U;
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    return AVR_OK;
}
/**
 * @brief Send START and the address/direction byte.
 * @param[in] address Valid unshifted seven-bit peripheral address.
 * @param[in] read True for peripheral-to-controller data direction.
 * @param[in] repeated True when the caller already owns an active transaction.
 * @param[in] budget Per-step unsuccessful poll budget.
 * @return Status of START or address acknowledgement; no STOP is generated here.
 * @details The public transaction validates arguments and performs error cleanup.
 */
static avr_status_t start(uint8_t address, bool read, bool repeated, uint32_t budget)
{
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
    avr_status_t result = wait_status(repeated ? TW_REP_START : TW_START, budget);
    if (result != AVR_OK)
    {
        return result;
    }
    TWDR = (uint8_t)((uint8_t)(address << 1U) | (read ? 1U : 0U));
    TWCR = _BV(TWINT) | _BV(TWEN);
    return wait_status(read ? TW_MR_SLA_ACK : TW_MT_SLA_ACK, budget);
}
/** @see twi.h for the authoritative API contract. */
avr_status_t avr_twi_init(uint32_t frequency)
{
    if ((frequency != 100000UL) && (frequency != 400000UL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / frequency) - 16UL) / 2UL);
    TWCR = _BV(TWEN);
    return AVR_OK;
}
/** @see twi.h for the authoritative API contract. */
avr_status_t avr_twi_transfer(uint8_t address, const uint8_t *tx, uint8_t tx_length, uint8_t *rx,
                              uint8_t rx_length, uint32_t budget)
{
    avr_status_t result;
    if ((address < 8U) || (address > 0x77U) || ((tx_length != 0U) && (tx == NULL)) ||
        ((rx_length != 0U) && (rx == NULL)))
    {
        return AVR_ERROR_ARGUMENT;
    }
    if ((TWCR & _BV(TWEN)) == 0U)
    {
        return AVR_ERROR_IO;
    }
    bool first_read = (tx_length == 0U) && (rx_length != 0U);
    result = start(address, first_read, false, budget);
    for (uint8_t i = 0U; (i < tx_length) && (result == AVR_OK); ++i)
    {
        TWDR = tx[i];
        TWCR = _BV(TWINT) | _BV(TWEN);
        result = wait_status(TW_MT_DATA_ACK, budget);
    }
    if ((result == AVR_OK) && (rx_length != 0U) && !first_read)
    {
        result = start(address, true, true, budget);
    }
    for (uint8_t i = 0U; (i < rx_length) && (result == AVR_OK); ++i)
    {
        bool last = i == (uint8_t)(rx_length - 1U);
        TWCR = (uint8_t)(_BV(TWINT) | _BV(TWEN) | (last ? 0U : _BV(TWEA)));
        result = wait_status(last ? TW_MR_DATA_NACK : TW_MR_DATA_ACK, budget);
        if (result == AVR_OK)
        {
            rx[i] = TWDR;
        }
    }
    if (result == AVR_ERROR_ARBITRATION)
    {
        TWCR = _BV(TWINT) | _BV(TWEN); /* Release bus; do not issue STOP as losing controller. */
        return result;
    }
    avr_status_t stopped = stop(budget);
    return (result == AVR_OK) ? stopped : result;
}
