/**
 * @file common.h
 * @brief Shared status codes and unsigned time arithmetic.
 * @details Defines the 16 MHz Uno R3 build contract. avr_elapsed is pure arithmetic and performs no
 * hardware access.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_COMMON_H
#define AVR_HAL_COMMON_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifndef F_CPU
/** @brief Reference Uno R3 CPU frequency in hertz, used by delay calculations. */
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL
#error "This release supports the 16 MHz Uno R3 only"
#endif
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Operation outcome; output validity is documented by each function. */
    typedef enum
    {
        AVR_OK = 0,            /**< Requested operation succeeded. */
        AVR_ERROR_ARGUMENT,    /**< Input validation failed before the operation. */
        AVR_ERROR_TIMEOUT,     /**< Poll/deadline bound reached; inspect API recovery policy. */
        AVR_ERROR_BUSY,        /**< Hardware or operation is already occupied. */
        AVR_ERROR_IO,          /**< Unexpected hardware state or transport error. */
        AVR_ERROR_NACK,        /**< Peripheral did not acknowledge a TWI address/data byte. */
        AVR_ERROR_ARBITRATION, /**< TWI controller lost arbitration to another controller. */
        AVR_ERROR_OVERFLOW,    /**< A bounded buffer or value could not hold further data. */
        AVR_ERROR_UNSUPPORTED /**< Valid input or device state is outside the implemented subset. */
    } avr_status_t;
/** @brief Default unsuccessful poll bound in iterations, not microseconds. */
#define AVR_POLL_BUDGET UINT32_C(100000)
    /**
     * @brief Test whether an unsigned tick interval has elapsed.
     * @param[in] now Current tick value in the same units as start and interval.
     * @param[in] start Saved starting tick; unsigned wraparound is allowed.
     * @param[in] interval Required interval, less than 2^31 ticks.
     * @return True when unsigned now-start is at least interval.
     * @details Pure calculation; no peripheral access. The caller must sample often enough to avoid
     * ambiguity after a complete counter wrap. Safe in any context with consistent input snapshots.
     */
    bool avr_elapsed(uint32_t now, uint32_t start, uint32_t interval);
#ifdef __cplusplus
}
#endif
#endif
