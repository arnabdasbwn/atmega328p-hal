/**
 * @file timer.c
 * @brief Independent Timer0, Timer1 and Timer2 configuration.
 * @details Implements timer-specific clock encodings and register layouts. Configuration owns the
 * complete selected timer and disables its interrupts. Application code explicitly selects any ISR
 * owner; no global interrupts are enabled here.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/timer.h"
#include <avr/io.h>
#include <util/atomic.h>
/**
 * @brief Map a numeric prescaler to the timer-specific CSn field.
 * @param[in] timer Timer identity; caller separately validates the enum.
 * @param[in] divider Requested numeric clock divisor.
 * @return Nonzero clock-select bits, or zero when the divisor is unsupported.
 * @details Timer2 has different encodings and supports divide-by-32/128.
 */
static uint8_t clock_bits(avr_timer_t timer, uint16_t divider)
{
    if (timer == AVR_TIMER2)
    {
        switch (divider)
        {
        case 1:
            return 1U;
        case 8:
            return 2U;
        case 32:
            return 3U;
        case 64:
            return 4U;
        case 128:
            return 5U;
        case 256:
            return 6U;
        case 1024:
            return 7U;
        default:
            return 0U;
        }
    }
    switch (divider)
    {
    case 1:
        return 1U;
    case 8:
        return 2U;
    case 64:
        return 3U;
    case 256:
        return 4U;
    case 1024:
        return 5U;
    default:
        return 0U;
    }
}
/** @see timer.h for the authoritative API contract. */
avr_status_t avr_timer_ctc(avr_timer_t timer, uint16_t prescaler, uint16_t top)
{
    uint8_t cs = clock_bits(timer, prescaler);
    if ((timer > AVR_TIMER2) || (timer < AVR_TIMER0) || (cs == 0U) ||
        ((timer != AVR_TIMER1) && (top > 255U)))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (timer == AVR_TIMER0)
        {
            TCCR0B = 0U;
            TIMSK0 = 0U;
            TCCR0A = _BV(WGM01);
            TCNT0 = 0U;
            OCR0A = (uint8_t)top;
            TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
            TCCR0B = cs;
        }
        else if (timer == AVR_TIMER1)
        {
            TCCR1B = 0U;
            TIMSK1 = 0U;
            TCCR1A = 0U;
            TCNT1 = 0U;
            OCR1A = top;
            TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
            TCCR1B = (uint8_t)(_BV(WGM12) | cs);
        }
        else
        {
            TCCR2B = 0U;
            TIMSK2 = 0U;
            TCCR2A = _BV(WGM21);
            TCNT2 = 0U;
            OCR2A = (uint8_t)top;
            TIFR2 = _BV(OCF2A) | _BV(OCF2B) | _BV(TOV2);
            TCCR2B = cs;
        }
    }
    return AVR_OK;
}
/** @see timer.h for the authoritative API contract. */
avr_status_t avr_timer_pwm8(avr_timer_t timer, uint16_t prescaler, uint8_t duty_a, uint8_t duty_b)
{
    uint8_t cs = clock_bits(timer, prescaler);
    if ((timer > AVR_TIMER2) || (timer < AVR_TIMER0) || (cs == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (timer == AVR_TIMER0)
        {
            TCCR0B = 0U;
            TIMSK0 = 0U;
            TCNT0 = 0U;
            OCR0A = duty_a;
            OCR0B = duty_b;
            DDRD |= _BV(DDD6) | _BV(DDD5);
            TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
            TCCR0B = cs;
        }
        else if (timer == AVR_TIMER1)
        {
            TCCR1B = 0U;
            TIMSK1 = 0U;
            TCNT1 = 0U;
            OCR1A = duty_a;
            OCR1B = duty_b;
            DDRB |= _BV(DDB1) | _BV(DDB2);
            TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
            TCCR1B = (uint8_t)(_BV(WGM12) | cs);
        }
        else
        {
            TCCR2B = 0U;
            TIMSK2 = 0U;
            TCNT2 = 0U;
            OCR2A = duty_a;
            OCR2B = duty_b;
            DDRB |= _BV(DDB3);
            DDRD |= _BV(DDD3);
            TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
            TCCR2B = cs;
        }
    }
    return AVR_OK;
}
/** @see timer.h for the authoritative API contract. */
avr_status_t avr_timer1_pwm(uint16_t prescaler, uint16_t period_ticks, uint16_t high_ticks)
{
    uint8_t cs = clock_bits(AVR_TIMER1, prescaler);
    if ((cs == 0U) || (period_ticks < 2U) || (high_ticks == 0U) || (high_ticks >= period_ticks))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCCR1B = 0U;
        TIMSK1 = 0U;
        TCCR1A = 0U;
        TCNT1 = 0U;
        ICR1 = (uint16_t)(period_ticks - 1U);
        OCR1A = high_ticks;
        DDRB |= _BV(DDB1);
        TCCR1A = _BV(COM1A1) | _BV(WGM11);
        TCCR1B = (uint8_t)(_BV(WGM13) | _BV(WGM12) | cs);
    }
    return AVR_OK;
}
/** @see timer.h for the authoritative API contract. */
avr_status_t avr_timer1_capture(uint16_t prescaler, bool rising)
{
    uint8_t cs = clock_bits(AVR_TIMER1, prescaler);
    if (cs == 0U)
    {
        return AVR_ERROR_ARGUMENT;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCCR1B = 0U;
        TIMSK1 = 0U;
        TCCR1A = 0U;
        TCNT1 = 0U;
        DDRB &= (uint8_t)~_BV(DDB0);
        TIFR1 = _BV(ICF1) | _BV(TOV1);
        TCCR1B = (uint8_t)(_BV(ICNC1) | (rising ? _BV(ICES1) : 0U) | cs);
    }
    return AVR_OK;
}
/** @see timer.h for the authoritative API contract. */
avr_status_t avr_timer_stop(avr_timer_t timer)
{
    switch (timer)
    {
    case AVR_TIMER0:
        TCCR0B = 0U;
        TIMSK0 = 0U;
        break;
    case AVR_TIMER1:
        TCCR1B = 0U;
        TIMSK1 = 0U;
        break;
    case AVR_TIMER2:
        TCCR2B = 0U;
        TIMSK2 = 0U;
        break;
    default:
        return AVR_ERROR_ARGUMENT;
    }
    return AVR_OK;
}
