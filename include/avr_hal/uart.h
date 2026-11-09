/**
 * @file uart.h
 * @brief Bounded polling USART0 transport.
 * @details Owns D0/RX and D1/TX in asynchronous 8N1 mode. One foreground caller owns the
 * peripheral; initialization disables USART interrupts without enabling global interrupts. Budgets
 * count polling iterations rather than elapsed time.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_UART_H
#define AVR_HAL_UART_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Configure USART0 for asynchronous 8N1.
     * @param[in] baud Requested baud rate, 300..1000000 bits/s.
     * @retval AVR_OK USART0 configured.
     * @retval AVR_ERROR_ARGUMENT Range invalid or nearest divisor error exceeds 3%; registers
     * unchanged.
     * @details Selects the lower absolute baud error of normal/double speed (normal wins ties).
     * Owns D0/RX and D1/TX; disables USART interrupts. Call before transfers with no concurrent
     * user. Does not enable global interrupts; nonblocking.
     */
    avr_status_t avr_uart_init(uint32_t baud);
    /**
     * @brief Send one byte when the transmit buffer is available.
     * @param[in] byte Eight data bits to enqueue.
     * @param[in] budget Maximum unsuccessful readiness polls; not a time in microseconds.
     * @retval AVR_OK Byte accepted into UDR0; physical transmission may still be in progress.
     * @retval AVR_ERROR_TIMEOUT Buffer did not become ready; byte not written.
     * @details Call after avr_uart_init from foreground only. Polling is bounded. Single caller
     * owns USART0; no simultaneous ISR transmitter. Zero budget permits an already-ready buffer.
     */
    avr_status_t avr_uart_put(uint8_t byte, uint32_t budget);
    /**
     * @brief Receive a byte and check its hardware error flags.
     * @param[out] byte Non-null destination; unchanged on all errors.
     * @param[in] budget Maximum unsuccessful readiness polls; iteration count, not time.
     * @retval AVR_OK Valid received byte stored.
     * @retval AVR_ERROR_ARGUMENT Null destination.
     * @retval AVR_ERROR_TIMEOUT No received byte became ready.
     * @retval AVR_ERROR_IO Framing, parity or overrun error; the erroneous byte was consumed.
     * @details Call after avr_uart_init from foreground, without another receiver. Reads status
     * before UDR0 because error flags belong to that buffered byte. Does not alter global interrupt
     * state.
     */
    avr_status_t avr_uart_get(uint8_t *byte, uint32_t budget);
    /**
     * @brief Send a bounded sequence of bytes.
     * @param[in] data Caller-owned buffer of at least length bytes; may be NULL only when length is
     * zero.
     * @param[in] length Number of bytes to send, in bytes.
     * @param[in] budget Readiness poll bound for each byte, not for the entire sequence.
     * @retval AVR_OK Every byte was accepted for transmission.
     * @retval AVR_ERROR_ARGUMENT Nonzero length with null buffer.
     * @retval AVR_ERROR_TIMEOUT Transmission stopped; a prefix may already have been sent.
     * @details Foreground, single-transmitter use after initialization. The buffer remains
     * caller-owned and is not retained after return. Does not wait for the final stop bit.
     */
    avr_status_t avr_uart_write(const uint8_t *data, size_t length, uint32_t budget);
    /**
     * @brief Print an unsigned value as decimal digits.
     * @param[in] value Value in 0..65535; no sign, leading zeros or newline is added.
     * @retval AVR_OK All decimal digits accepted for transmission.
     * @retval AVR_ERROR_TIMEOUT A prefix may have been transmitted before timeout.
     * @details Foreground only after UART initialization. Uses five stack bytes for digits, no heap
     * and no printf. Each byte uses AVR_POLL_BUDGET.
     */
    avr_status_t avr_uart_u16(uint16_t value);
#ifdef __cplusplus
}
#endif
#endif
