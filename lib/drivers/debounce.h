/**
 * @file debounce.h
 * @brief Polled input debounce state machine.
 * @details Separates the sampled candidate level from the accepted stable level. The state records
 * a millisecond timestamp; a changed candidate restarts the quiet interval. No delays or peripheral
 * operations occur in this driver.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DEBOUNCE_H
#define AVR_DEBOUNCE_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief State of one polled input; one execution context owns updates. */
    typedef struct
    {
        bool stable;    /**< Last accepted electrical level. */
        bool candidate; /**< Most recently observed candidate level. */
        uint32_t since; /**< Millisecond timestamp when candidate last changed. */
    } avr_debounce_t;
    /**
     * @brief Initialize a sampled-level debounce state.
     * @param[out] button State object; NULL is accepted as a no-op.
     * @param[in] initial Initial stable and candidate electrical level.
     * @param[in] now Initial millisecond timestamp.
     * @details Pure state initialization, no hardware access. The caller owns state lifetime and
     * synchronization. Initialize before avr_debounce_update.
     */
    void avr_debounce_init(avr_debounce_t *button, bool initial, uint32_t now);
    /**
     * @brief Accept a stable level after a quiet interval.
     * @param[in,out] button Initialized state object; NULL returns false.
     * @param[in] sample Current electrical level, not interpreted as pressed/released.
     * @param[in] now Monotonic millisecond tick with unsigned wraparound.
     * @param[in] interval_ms Required unchanged duration, 0..65535 ms.
     * @return True once when stable level changes; false otherwise.
     * @details Nonblocking, no register access. A candidate change restarts its timestamp. One
     * caller owns each state. Poll often enough to observe input transitions; unobserved bounce
     * cannot be reconstructed.
     */
    bool avr_debounce_update(avr_debounce_t *button, bool sample, uint32_t now,
                             uint16_t interval_ms);
#ifdef __cplusplus
}
#endif
#endif
