/**
 * @file common.c
 * @brief Shared status codes and unsigned time arithmetic.
 * @details Defines the 16 MHz Uno R3 build contract. avr_elapsed is pure arithmetic and performs no
 * hardware access.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/common.h"
/** @see common.h for the authoritative API contract. */
bool avr_elapsed(uint32_t now, uint32_t start, uint32_t interval)
{
    return (uint32_t)(now - start) >= interval;
}
