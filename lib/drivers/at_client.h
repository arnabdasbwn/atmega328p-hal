/**
 * @file at_client.h
 * @brief Bounded polling helpers for simple UART AT-command modules.
 * @details Uses the initialized AVR HAL USART. Callers provide command bytes and response tokens;
 * the driver allocates no heap memory and stores no global state.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_AT_CLIENT_H
#define AVR_DRIVER_AT_CLIENT_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Maximum response-token length retained by the stack-based prefix table. */
#define AVR_AT_MAX_TOKEN_LENGTH UINT8_C(32)

    /**
     * @brief Send caller-provided AT command bytes without appending a terminator.
     * @param[in] command Caller-owned byte sequence; NULL is allowed only when length is zero.
     * @param[in] length Number of bytes to send.
     * @return AVR status from the HAL USART, including timeout and argument errors.
     * @details Requires initialized UART. The caller selects command terminators such as CR/LF and
     * owns command formatting. Each transmitted byte uses the supplied polling budget.
     */
    avr_status_t avr_at_send(const uint8_t *command, size_t length);

    /**
     * @brief Read a bounded number of UART bytes until a response token is matched.
     * @param[in] token Caller-owned exact byte sequence, length 1..AVR_AT_MAX_TOKEN_LENGTH.
     * @param[in] token_length Number of token bytes; no NUL terminator is required.
     * @param[in] maximum_bytes Maximum received bytes to inspect before returning timeout, 1..255.
     * @param[in] polls_per_byte HAL UART receive polling budget for each byte, not elapsed time.
     * @return AVR_OK when token appears, AVR_ERROR_ARGUMENT for invalid bounds, AVR_ERROR_TIMEOUT
     * when no match appears within the receive bound, or another HAL UART status.
     * @details Requires initialized UART and exclusive foreground receive ownership. Uses a fixed
     * 32-byte prefix table on the stack to match tokens across byte boundaries. Bytes read before a
     * match are consumed and not retained; no buffer is returned. Matching is case-sensitive.
     */
    avr_status_t avr_at_wait_token(const uint8_t *token, uint8_t token_length,
                                   uint8_t maximum_bytes, uint32_t polls_per_byte);
#ifdef __cplusplus
}
#endif
#endif
