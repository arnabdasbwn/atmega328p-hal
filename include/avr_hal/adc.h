/**
 * @file adc.h
 * @brief Right-adjusted ADC sampling for Uno analog inputs.
 * @details Supports A0..A5 with a 125 kHz ADC clock at F_CPU=16 MHz. Reference and channel changes
 * require settling. The polling API is foreground-only and the application must not run a competing
 * ADC ISR.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_ADC_H
#define AVR_HAL_ADC_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief ADC reference selection encoded for ADMUX REFS1:0. */
    typedef enum
    {
        AVR_ADC_AREF = 0,        /**< External reference applied to AREF. */
        AVR_ADC_AVCC = 1,        /**< AVCC reference; decouple AREF without driving it. */
        AVR_ADC_INTERNAL_1V1 = 3 /**< Nominal internal 1.1 V reference; calibrate actual value. */
    } avr_adc_reference_t;
    /**
     * @brief Enable right-adjusted ADC conversions at 125 kHz.
     * @param[in] reference External AREF, AVCC or nominal internal 1.1 V reference.
     * @retval AVR_OK ADC configured and initial reference delay completed.
     * @retval AVR_ERROR_ARGUMENT Unsupported reference; registers unchanged.
     * @details Owns ADC configuration and clears ADCSRB (including comparator multiplexing/trigger
     * selection). Disables ADC interrupts and auto-trigger. Blocks about 2 ms to settle reference;
     * foreground only. Do not externally drive AREF when AVCC/internal reference is selected.
     * Global interrupts unchanged.
     */
    avr_status_t avr_adc_init(avr_adc_reference_t reference);
    /**
     * @brief Measure a Uno analog channel with bounded polling.
     * @param[in] channel ADC channel 0..5, corresponding to A0..A5.
     * @param[out] sample Non-null destination for 0..1023; unchanged on errors.
     * @param[in] budget Maximum unsuccessful polls per conversion, not an elapsed-time deadline.
     * @retval AVR_OK Measurement stored.
     * @retval AVR_ERROR_ARGUMENT Invalid channel or null destination.
     * @retval AVR_ERROR_IO ADC is disabled.
     * @retval AVR_ERROR_BUSY A conversion is already running.
     * @retval AVR_ERROR_TIMEOUT Conversion did not finish; hardware may still be converting.
     * @details Foreground, sole ADC owner after avr_adc_init. Disables the selected digital input
     * buffer. Discards one conversion after initialization or channel changes; source settling
     * still depends on circuit impedance. No global interrupt changes.
     */
    avr_status_t avr_adc_read(uint8_t channel, uint16_t *sample, uint32_t budget);
    /**
     * @brief Scale an ideal ADC code into millivolts.
     * @param[in] sample ADC result; values above 1023 are clamped to 1023.
     * @param[in] reference_mv Calibrated reference voltage in millivolts, 0..65535.
     * @return Rounded sample*reference_mv/1024 in millivolts.
     * @details Pure, bounded arithmetic using a 32-bit intermediate. Does not calibrate the ADC or
     * infer reference accuracy. Uses 1024 quantization bins, not endpoint interpolation.
     */
    uint16_t avr_adc_millivolts(uint16_t sample, uint16_t reference_mv);
#ifdef __cplusplus
}
#endif
#endif
