/**
 * @file spi.c
 * @brief MSB-first SPI controller transfers.
 * @details Owns D10/SS, D11/COPI, D12/CIPO and D13/SCK. Peripheral chip select remains the
 * application responsibility. Controller mode requires SS configured as an output. Transfers are
 * bounded and nonreentrant.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "avr_hal/spi.h"
#include <avr/io.h>
/** @see spi.h for the authoritative API contract. */
avr_status_t avr_spi_init(uint8_t mode, uint16_t divider)
{
    uint8_t spr, twice;
    if (mode > 3U)
    {
        return AVR_ERROR_ARGUMENT;
    }
    switch (divider)
    {
    case 2:
        spr = 0U;
        twice = 1U;
        break;
    case 4:
        spr = 0U;
        twice = 0U;
        break;
    case 8:
        spr = 1U;
        twice = 1U;
        break;
    case 16:
        spr = 1U;
        twice = 0U;
        break;
    case 32:
        spr = 2U;
        twice = 1U;
        break;
    case 64:
        spr = 2U;
        twice = 0U;
        break;
    case 128:
        spr = 3U;
        twice = 0U;
        break;
    default:
        return AVR_ERROR_ARGUMENT;
    }
    PORTB |= _BV(PORTB2); /* SS must stay an output in controller mode. */
    if ((mode & 2U) != 0U)
    {
        PORTB |= _BV(PORTB5);
    }
    else
    {
        PORTB &= (uint8_t)~_BV(PORTB5);
    }
    DDRB |= _BV(DDB2) | _BV(DDB3) | _BV(DDB5);
    DDRB &= (uint8_t)~_BV(DDB4);
    SPCR = (uint8_t)(_BV(SPE) | _BV(MSTR) | (uint8_t)(mode << 2U) | spr);
    SPSR = twice;
    return AVR_OK;
}
/** @see spi.h for the authoritative API contract. */
avr_status_t avr_spi_transfer(uint8_t transmit, uint8_t *receive, uint32_t budget)
{
    uint8_t status;
    if (receive == NULL)
    {
        return AVR_ERROR_ARGUMENT;
    }
    if ((SPCR & (_BV(SPE) | _BV(MSTR))) != (_BV(SPE) | _BV(MSTR)))
    {
        return AVR_ERROR_IO;
    }
    SPDR = transmit;
    while ((SPSR & _BV(SPIF)) == 0U)
    {
        if (budget == 0UL)
        {
            return AVR_ERROR_TIMEOUT;
        }
        --budget;
    }
    status = SPSR;
    uint8_t received = SPDR; /* Status then data clears SPIF/WCOL. */
    if ((status & _BV(WCOL)) != 0U)
    {
        return AVR_ERROR_BUSY;
    }
    *receive = received;
    return AVR_OK;
}
