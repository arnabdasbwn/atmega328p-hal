/**
 * @file debounce.c
 * @brief Polled input debounce state machine.
 * @details Separates the sampled candidate level from the accepted stable level. The state records
 * a millisecond timestamp; a changed candidate restarts the quiet interval. No delays or peripheral
 * operations occur in this driver.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "debounce.h"
/** @see debounce.h for the authoritative API contract. */
void avr_debounce_init(avr_debounce_t *button, bool initial, uint32_t now)
{
    if (button != NULL)
    {
        button->stable = initial;
        button->candidate = initial;
        button->since = now;
    }
}
/** @see debounce.h for the authoritative API contract. */
bool avr_debounce_update(avr_debounce_t *button, bool sample, uint32_t now, uint16_t interval_ms)
{
    if (button == NULL)
    {
        return false;
    }
    if (sample != button->candidate)
    {
        button->candidate = sample;
        button->since = now;
    }
    if ((button->candidate != button->stable) && avr_elapsed(now, button->since, interval_ms))
    {
        button->stable = button->candidate;
        return true;
    }
    return false;
}
