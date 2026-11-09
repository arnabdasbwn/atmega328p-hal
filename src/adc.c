/**
 * @file adc.c
 * @brief Right-adjusted ADC sampling for Uno analog inputs.
 * @details Supports A0..A5 with a 125 kHz ADC clock at F_CPU=16 MHz. Reference and channel changes
 * require settling. The polling API is foreground-only and the application must not run a competing
 * ADC ISR.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/adc.h"
#include <avr/io.h>
#include <util/delay.h>
/** @brief Last settled channel; UINT8_MAX forces a discarded first conversion. Foreground-owned. */
static uint8_t previous_channel = UINT8_MAX;
/** @see adc.h for the authoritative API contract. */
avr_status_t avr_adc_init(avr_adc_reference_t reference)
{
    if ((reference != AVR_ADC_AREF) && (reference != AVR_ADC_AVCC) &&
        (reference != AVR_ADC_INTERNAL_1V1))
    {
        return AVR_ERROR_ARGUMENT;
    }
    ADMUX = (uint8_t)((uint8_t)reference << 6U);
    ADCSRB = 0U;
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    previous_channel = UINT8_MAX;
    _delay_ms(2); /* Allow reference to settle; source impedance still matters. */
    return AVR_OK;
}
/**
 * @brief Start and wait for one right-adjusted conversion.
 * @param[out] sample Valid internal result pointer; untouched on timeout.
 * @param[in] budget Maximum unsuccessful ADSC polls, in iterations.
 * @retval AVR_OK Conversion completed and result was read low-byte first through ADC.
 * @retval AVR_ERROR_TIMEOUT Conversion remains pending beyond the polling budget.
 * @details Caller validates ADC ownership. Do not clear ADIF accidentally while setting ADSC.
 */
static avr_status_t convert(uint16_t *sample, uint32_t budget)
{
    ADCSRA = (uint8_t)((ADCSRA & (uint8_t)~_BV(ADIF)) | _BV(ADSC));
    while ((ADCSRA & _BV(ADSC)) != 0U)
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    *sample = ADC;
    return AVR_OK;
}
/** @see adc.h for the authoritative API contract. */
avr_status_t avr_adc_read(uint8_t channel, uint16_t *sample, uint32_t budget)
{
    uint16_t discarded;
    if ((channel > 5U) || (sample == NULL))
    {
        return AVR_ERROR_ARGUMENT;
    }
    if ((ADCSRA & _BV(ADEN)) == 0U)
    {
        return AVR_ERROR_IO;
    }
    if ((ADCSRA & _BV(ADSC)) != 0U)
    {
        return AVR_ERROR_BUSY;
    }
    ADMUX = (uint8_t)((ADMUX & 0xF0U) | channel);
    DIDR0 = (uint8_t)(DIDR0 | (uint8_t)(UINT8_C(1) << channel));
    if (previous_channel != channel)
    {
        avr_status_t status = convert(&discarded, budget);
        if (status != AVR_OK)
        {
            return status;
        }
        previous_channel = channel;
    }
    return convert(sample, budget);
}
/** @see adc.h for the authoritative API contract. */
uint16_t avr_adc_millivolts(uint16_t sample, uint16_t reference_mv)
{
    if (sample > 1023U)
    {
        sample = 1023U;
    }
    return (uint16_t)(((uint32_t)sample * reference_mv + 512UL) / 1024UL);
}
