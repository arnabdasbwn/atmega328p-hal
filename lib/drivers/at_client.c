/**
 * @file at_client.c
 * @brief UART AT-command transmission and bounded token matching.
 * @details Builds a fixed-size Knuth-Morris-Pratt prefix table for the requested response token,
 * then consumes no more than the caller's receive-byte limit.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "at_client.h"
#include "avr_hal/uart.h"

/** @see at_client.h for command ownership, terminator and timeout contracts. */
avr_status_t avr_at_send(const uint8_t *command, size_t length)
{
    if ((command == NULL) && (length != 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    return avr_uart_write(command, length, AVR_POLL_BUDGET);
}

/** @see at_client.h for token bounds, receive consumption and timeout contracts. */
avr_status_t avr_at_wait_token(const uint8_t *token, uint8_t token_length, uint8_t maximum_bytes,
                               uint32_t polls_per_byte)
{
    uint8_t prefix[AVR_AT_MAX_TOKEN_LENGTH];
    uint8_t matched = 0U;
    if ((token == NULL) || (token_length == 0U) || (token_length > AVR_AT_MAX_TOKEN_LENGTH) ||
        (maximum_bytes == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    prefix[0] = 0U;
    for (uint8_t index = 1U; index < token_length; index++)
    {
        uint8_t candidate = prefix[index - 1U];
        while ((candidate > 0U) && (token[index] != token[candidate]))
        {
            candidate = prefix[candidate - 1U];
        }
        if (token[index] == token[candidate])
        {
            candidate++;
        }
        prefix[index] = candidate;
    }
    for (uint16_t received_count = 0U; received_count < maximum_bytes; received_count++)
    {
        uint8_t byte;
        avr_status_t status = avr_uart_get(&byte, polls_per_byte);
        if (status != AVR_OK)
        {
            return status;
        }
        while ((matched > 0U) && (byte != token[matched]))
        {
            matched = prefix[matched - 1U];
        }
        if (byte == token[matched])
        {
            matched++;
        }
        if (matched == token_length)
        {
            return AVR_OK;
        }
    }
    return AVR_ERROR_TIMEOUT;
}
