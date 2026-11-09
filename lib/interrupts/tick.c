/**
 * @file tick.c
 * @brief Optional Timer0 compare-A millisecond timebase.
 * @details Owns Timer0 and TIMER0_COMPA_vect. A divide-by-64 clock and OCR0A=249 give 1 kHz. The
 * ISR updates shared state; foreground snapshots use restore-state atomic blocks. Do not combine
 * with the Arduino core Timer0 owner.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/tick.h"
#include "avr_hal/timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
/** @brief Millisecond count written by TIMER0_COMPA ISR; foreground reads atomically. */
static volatile uint32_t ticks;
/** @see tick.h for the authoritative API contract. */
void avr_tick_init(void)
{
    (void)avr_timer_ctc(AVR_TIMER0, 64U, 249U);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ticks = 0UL;
        TIMSK0 = _BV(OCIE0A);
    }
}
/** @see tick.h for the authoritative API contract. */
uint32_t avr_tick_ms(void)
{
    uint32_t result;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = ticks;
    }
    return result;
}
/**
 * @brief Count a Timer0 compare-A event without blocking.
 * @details Sole owner of TIMER0_COMPA_vect. Advances ticks modulo 2^32.
 * Multiple events while interrupts are masked collapse into one pending flag.
 */
ISR(TIMER0_COMPA_vect)
{
    ++ticks;
}
