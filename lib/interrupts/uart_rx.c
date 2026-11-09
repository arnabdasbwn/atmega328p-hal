/**
 * @file uart_rx.c
 * @brief Bounded single-producer/single-consumer USART receive queue.
 * @details Owns USART_RX_vect. All published ring accesses are volatile byte
 * operations; the ISR writes payload before head, and foreground reads payload
 * before advancing tail. One unused slot distinguishes full from empty.
 * No delay, print or bus polling occurs in the ISR. This is an AVR-specific
 * foreground/ISR protocol, not a portable multithreaded lock-free C queue.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/uart_rx.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
/** @brief Queued bytes; ISR writes, foreground reads before releasing each slot. */
static volatile uint8_t bytes[AVR_UART_RX_SLOTS];
/** @brief Next producer slot, written only by RX ISR between start operations. */
static volatile uint8_t head;
/** @brief Oldest unread slot, written only by foreground consumer. */
static volatile uint8_t tail;
/** @brief Saturating full-queue loss count; ISR writer, atomic snapshot reader. */
static volatile uint16_t dropped;
/** @brief Saturating hardware-error count; ISR writer, atomic snapshot reader. */
static volatile uint16_t errors;

/**
 * @brief Consume exactly one receive frame and publish it or record its loss.
 * @details Read status before UDR0 because the flags belong to the current
 * received frame. Bad frames are discarded. Full queues keep their older data
 * and discard the new byte. Global interrupt nesting is not enabled here.
 */
ISR(USART_RX_vect)
{
    uint8_t status = UCSR0A;
    uint8_t value = UDR0;
    if ((status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) != 0U)
    {
        if (errors != UINT16_MAX)
        {
            ++errors;
        }
        return;
    }
    uint8_t next = (uint8_t)((head + 1U) & (AVR_UART_RX_SLOTS - 1U));
    if (next == tail)
    {
        if (dropped != UINT16_MAX)
        {
            ++dropped;
        }
        return;
    }
    bytes[head] = value;
    head = next; /* Publish only after the payload is stored. */
}

/** @see uart_rx.h for the authoritative API contract. */
avr_status_t avr_uart_rx_start(void)
{
    if ((UCSR0B & _BV(RXEN0)) == 0U)
    {
        return AVR_ERROR_BUSY;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        head = 0U;
        tail = 0U;
        dropped = 0U;
        errors = 0U;
        UCSR0B |= _BV(RXCIE0);
    }
    return AVR_OK;
}

/** @see uart_rx.h for the authoritative API contract. */
avr_status_t avr_uart_rx_get(uint8_t *byte)
{
    if (byte == NULL)
    {
        return AVR_ERROR_ARGUMENT;
    }
    uint8_t position = tail;
    if (position == head)
    {
        return AVR_ERROR_BUSY;
    }
    *byte = bytes[position];
    tail = (uint8_t)((position + 1U) & (AVR_UART_RX_SLOTS - 1U));
    return AVR_OK;
}

/** @see uart_rx.h for the authoritative API contract. */
avr_status_t avr_uart_rx_stats(avr_uart_rx_stats_t *stats)
{
    if (stats == NULL)
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        stats->dropped = dropped;
        stats->errors = errors;
    }
    return AVR_OK;
}
