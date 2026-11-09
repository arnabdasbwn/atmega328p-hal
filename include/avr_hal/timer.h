/**
 * @file timer.h
 * @brief Independent Timer0, Timer1 and Timer2 configuration.
 * @details Implements timer-specific clock encodings and register layouts. Configuration owns the
 * complete selected timer and disables its interrupts. Application code explicitly selects any ISR
 * owner; no global interrupts are enabled here.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_TIMER_H
#define AVR_HAL_TIMER_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief ATmega328P timer peripheral identity. */
    typedef enum
    {
        AVR_TIMER0, /**< Eight-bit timer with OC0A/D6 and OC0B/D5. */
        AVR_TIMER1, /**< Sixteen-bit timer with OC1A/D9, OC1B/D10 and ICP1/D8. */
        AVR_TIMER2  /**< Eight-bit timer with separate prescaler encodings, D11/D3 outputs. */
    } avr_timer_t;
    /**
     * @brief Configure a timer for clear-on-compare operation.
     * @param[in] timer AVR_TIMER0, AVR_TIMER1 or AVR_TIMER2.
     * @param[in] prescaler Timer0/1: 1,8,64,256,1024. Timer2 also supports 32 and 128.
     * @param[in] top OCRnA value, 0..255 for timer0/2 or 0..65535 for timer1.
     * @retval AVR_OK Timer configured and running.
     * @retval AVR_ERROR_ARGUMENT Unsupported timer/divider/TOP; registers unchanged.
     * @details Owns the entire selected timer: disables its interrupts, resets counter and clears
     * pending flags. Compare outputs remain disconnected. Event rate is F_CPU/(prescaler*(top+1)).
     * Restores previous global interrupt state.
     */
    avr_status_t avr_timer_ctc(avr_timer_t timer, uint16_t prescaler, uint16_t top);
    /**
     * @brief Configure noninverting eight-bit fast PWM on both outputs.
     * @param[in] timer AVR_TIMER0, AVR_TIMER1 or AVR_TIMER2.
     * @param[in] prescaler Clock divisor supported by the selected timer.
     * @param[in] duty_a OCnA compare value, 0..255; endpoint behavior follows the datasheet.
     * @param[in] duty_b OCnB compare value, 0..255; endpoint behavior follows the datasheet.
     * @retval AVR_OK Timer and both output pins configured.
     * @retval AVR_ERROR_ARGUMENT Invalid timer/divider; registers unchanged.
     * @details Owns timer0 D6/D5, timer1 D9/D10 or timer2 D11/D3. Disables selected timer
     * interrupts. Fixed TOP=255; frequency F_CPU/(prescaler*256). OCR=0 can yield a narrow pulse,
     * not ideal zero duty. Restores interrupt state.
     */
    avr_status_t avr_timer_pwm8(avr_timer_t timer, uint16_t prescaler, uint8_t duty_a,
                                uint8_t duty_b);
    /**
     * @brief Generate variable-period noninverting PWM on D9.
     * @param[in] prescaler Timer1 divisor: 1,8,64,256 or 1024.
     * @param[in] period_ticks Complete frame length in prescaled ticks, 2..65535;
     * ICR1=period_ticks-1.
     * @param[in] high_ticks Pulse compare value in ticks, strictly between zero and period_ticks.
     * @retval AVR_OK Timer1 mode 14 and OC1A/D9 configured.
     * @retval AVR_ERROR_ARGUMENT Unsupported divider/period/pulse; registers unchanged.
     * @details Owns Timer1 and D9; disables Timer1 interrupts and disconnects OC1B. Restores global
     * interrupt state. At divider 8 a tick is 0.5 us. Stop timer before changing ICR1 if safe
     * dynamic TOP handling is not implemented.
     */
    avr_status_t avr_timer1_pwm(uint16_t prescaler, uint16_t period_ticks, uint16_t high_ticks);
    /**
     * @brief Start Timer1 normal-mode input capture.
     * @param[in] prescaler Timer1 divisor: 1,8,64,256 or 1024.
     * @param[in] rising True captures rising edges; false falling edges.
     * @retval AVR_OK Timer1 and ICP1/D8 configured.
     * @retval AVR_ERROR_ARGUMENT Unsupported divider; registers unchanged.
     * @details Enables hardware noise cancellation; clears capture/overflow flags and disables
     * Timer1 interrupts. Application owns any ISR and overflow accounting. Does not alter D8
     * pull-up latch or global interrupt state.
     */
    avr_status_t avr_timer1_capture(uint16_t prescaler, bool rising);
    /**
     * @brief Stop a timer clock and disable its interrupts.
     * @param[in] timer AVR_TIMER0, AVR_TIMER1 or AVR_TIMER2.
     * @retval AVR_OK Timer stopped.
     * @retval AVR_ERROR_ARGUMENT Invalid timer; registers unchanged.
     * @details Leaves output-compare configuration and GPIO latches in their current state; this is
     * not an actuator-safe shutdown. For safe loads explicitly disconnect outputs and drive
     * inactive levels. Use while no concurrent configuration writer is active.
     */
    avr_status_t avr_timer_stop(avr_timer_t timer);
#ifdef __cplusplus
}
#endif
#endif
