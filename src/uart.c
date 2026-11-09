/**
 * @file uart.c
 * @brief Bounded polling USART0 transport.
 * @details Owns D0/RX and D1/TX in asynchronous 8N1 mode. One foreground caller owns the
 * peripheral; initialization disables USART interrupts without enabling global interrupts. Budgets
 * count polling iterations rather than elapsed time.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/uart.h"
#include <avr/io.h>
/** @see uart.h for the authoritative API contract. */
avr_status_t avr_uart_init(uint32_t baud)
{
    uint32_t best_error = UINT32_MAX;
    uint16_t best_divisor = 0U;
    uint8_t best_speed = 0U;
    if ((baud < 300UL) || (baud > 1000000UL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    for (uint8_t speed = 0U; speed < 2U; ++speed)
    {
        uint32_t scale = (speed == 0U) ? 16UL : 8UL;
        uint32_t counts = (F_CPU + ((scale * baud) / 2UL)) / (scale * baud);
        if ((counts > 0UL) && (counts <= 4096UL))
        {
            uint32_t actual = F_CPU / (scale * counts);
            uint32_t error = (actual > baud) ? actual - baud : baud - actual;
            if (error < best_error)
            {
                best_error = error;
                best_divisor = (uint16_t)(counts - 1UL);
                best_speed = speed;
            }
        }
    }
    if ((best_error == UINT32_MAX) || (best_error > (baud * 3UL) / 100UL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    UCSR0B = 0U;
    UCSR0A = (best_speed != 0U) ? _BV(U2X0) : 0U;
    UBRR0H = (uint8_t)(best_divisor >> 8U);
    UBRR0L = (uint8_t)best_divisor;
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    return AVR_OK;
}
/** @see uart.h for the authoritative API contract. */
avr_status_t avr_uart_put(uint8_t byte, uint32_t budget)
{
    while ((UCSR0A & _BV(UDRE0)) == 0U)
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    UDR0 = byte;
    return AVR_OK;
}
/** @see uart.h for the authoritative API contract. */
avr_status_t avr_uart_get(uint8_t *byte, uint32_t budget)
{
    uint8_t status, data;
    if (byte == NULL)
    {
        return AVR_ERROR_ARGUMENT;
    }
    while ((UCSR0A & _BV(RXC0)) == 0U)
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    status = UCSR0A;
    data = UDR0;
    if ((status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) != 0U)
    {
        return AVR_ERROR_IO;
    }
    *byte = data;
    return AVR_OK;
}
/** @see uart.h for the authoritative API contract. */
avr_status_t avr_uart_write(const uint8_t *data, size_t length, uint32_t budget)
{
    if ((data == NULL) && (length != 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    for (size_t i = 0U; i < length; ++i)
    {
        avr_status_t status = avr_uart_put(data[i], budget);
        if (status != AVR_OK)
        {
            return status;
        }
    }
    return AVR_OK;
}
/** @see uart.h for the authoritative API contract. */
avr_status_t avr_uart_u16(uint16_t value)
{
    uint8_t digits[5];
    uint8_t count = 0U;
    do
    {
        digits[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count != 0U)
    {
        avr_status_t status = avr_uart_put(digits[--count], AVR_POLL_BUDGET);
        if (status != AVR_OK)
        {
            return status;
        }
    }
    return AVR_OK;
}
