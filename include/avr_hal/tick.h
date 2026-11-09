/**
 * @file tick.h
 * @brief Optional Timer0 compare-A millisecond timebase.
 * @details Owns Timer0 and TIMER0_COMPA_vect. A divide-by-64 clock and OCR0A=249 give 1 kHz. The
 * ISR updates shared state; foreground snapshots use restore-state atomic blocks. Do not combine
 * with the Arduino core Timer0 owner.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_TICK_H
#define AVR_HAL_TICK_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Start the optional one-millisecond Timer0 tick.
     * @details Owns Timer0 and TIMER0_COMPA_vect, resets the software counter and enables compare-A
     * interrupt. Uses divisor 64 and TOP 249. Application must enable global interrupts after
     * initialization. Cannot coexist with another Timer0 owner or the Arduino core timebase.
     */
    void avr_tick_init(void);
    /**
     * @brief Read an atomic millisecond counter snapshot.
     * @return Milliseconds counted by Timer0 compare-A ISR, modulo 2^32.
     * @details Requires avr_tick_init and running interrupts for progress. Uses a short
     * restore-state critical section to avoid torn 32-bit reads on an 8-bit CPU. Long interrupt
     * masking loses tick events; this is not a wall-clock guarantee.
     */
    uint32_t avr_tick_ms(void);
#ifdef __cplusplus
}
#endif
#endif
