/**
 * @file uart_rx.h
 * @brief Optional USART0 receive-interrupt queue for Uno R3.
 * @details Link lib/interrupts/uart_rx.c explicitly. It owns USART_RX_vect and
 * the UDR0 receive path. Do not also call avr_uart_get or install another RX ISR.
 * Foreground polling transmission remains available through uart.h.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_UART_RX_H
#define AVR_HAL_UART_RX_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
/** @brief Physical ring slots; one slot is reserved, giving 63 queued bytes. */
#define AVR_UART_RX_SLOTS 64U
    /** @brief Saturating receive-loss counters since the last successful start. */
    typedef struct
    {
        uint16_t dropped; /**< New bytes discarded because the software queue was full. */
        uint16_t errors;  /**< Received frames discarded for FE, UPE or hardware DOR. */
    } avr_uart_rx_stats_t;
    /**
     * @brief Empty the queue and enable the USART receive-complete interrupt.
     * @retval AVR_OK Queue reset and RX interrupt enabled.
     * @retval AVR_ERROR_BUSY UART receiver is disabled; no queue/register changes.
     * @details Call after avr_uart_init, from foreground with exclusive UART control.
     * Discards queued software bytes and resets counters; a pending hardware byte
     * may subsequently enter the new queue. Restores previous global interrupt state,
     * so the application must call sei when ready. Does not change baud or TX mode.
     */
    avr_status_t avr_uart_rx_start(void);
    /**
     * @brief Take the oldest queued byte without waiting for input.
     * @param[out] byte Non-null result pointer; unchanged unless AVR_OK is returned.
     * @retval AVR_OK One byte removed and stored in byte.
     * @retval AVR_ERROR_ARGUMENT Null output pointer.
     * @retval AVR_ERROR_BUSY Queue empty; retry later from the foreground loop.
     * @details Single foreground consumer only, after start. The ISR is the sole
     * producer. No global interrupt changes or poll loops occur. Queue overflow
     * drops new bytes; inspect stats to detect loss instead of assuming a full stream.
     */
    avr_status_t avr_uart_rx_get(uint8_t *byte);
    /**
     * @brief Snapshot loss counters consistently.
     * @param[out] stats Non-null caller-owned structure, overwritten on success.
     * @retval AVR_OK Both counters copied under one short restore-state critical section.
     * @retval AVR_ERROR_ARGUMENT Null pointer; nothing changed.
     * @details Foreground use after start. Counters saturate at 65535 and are not
     * cleared by reading. A DOR event means hardware lost data; it is not an exact
     * count of all missing bytes. Restarting resets both counters and the queue.
     */
    avr_status_t avr_uart_rx_stats(avr_uart_rx_stats_t *stats);
#ifdef __cplusplus
}
#endif
#endif
