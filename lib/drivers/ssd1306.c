/**
 * @file ssd1306.c
 * @brief Bounded page transfers for the SSD1306 display controller.
 * @details Uses only the public AVR HAL TWI interface and has no static framebuffer. The supported
 * initialization is for common 128x64 I2C panels; reset, exact glass orientation and module power
 * circuitry vary between breakouts. All transactions are synchronous and bounded by the caller's
 * per-state poll budget.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "drivers/ssd1306.h"
#include "avr_hal/twi.h"

/**
 * @brief Send one SSD1306 command byte.
 * @param[in] address Unshifted 7-bit I2C address.
 * @param[in] command Command byte to transmit.
 * @param[in] budget Maximum unsuccessful polls for each TWI protocol state.
 * @retval AVR_OK Command was acknowledged and STOP completed.
 * @retval AVR_ERROR_ARGUMENT Invalid transfer arguments.
 * @retval AVR_ERROR_NACK Target did not acknowledge.
 * @retval AVR_ERROR_TIMEOUT A TWI state or STOP timed out.
 * @retval AVR_ERROR_IO TWI is unavailable or returned unexpected status.
 * @retval AVR_ERROR_ARBITRATION Bus arbitration was lost; TWI releases without STOP.
 * @details The 0x00 control byte selects command mode. Foreground only; no retry is attempted.
 */
static avr_status_t send_command(uint8_t address, uint8_t command, uint32_t budget)
{
    const uint8_t packet[2] = {0x00U, command};
    return avr_twi_transfer(address, packet, (uint8_t)sizeof(packet), NULL, 0U, budget);
}

avr_status_t avr_ssd1306_init(uint8_t address, uint32_t budget)
{
    static const uint8_t commands[] = {
        0xAEU,        /* Display off while configuration is changing. */
        0xD5U, 0x80U, /* Clock divide and oscillator frequency. */
        0xA8U, 0x3FU, /* 64-row multiplex ratio. */
        0xD3U, 0x00U, /* No display offset. */
        0x40U,        /* Start line zero. */
        0x8DU, 0x14U, /* Enable the internal charge pump. */
        0x20U, 0x02U, /* Page addressing mode. */
        0xA1U,        /* Segment remap. */
        0xC8U,        /* COM scan direction. */
        0xDAU, 0x12U, /* Common 128x64 panel pin configuration. */
        0x81U, 0x7FU, /* Moderate contrast. */
        0xD9U, 0xF1U, /* Pre-charge period for internal supply. */
        0xDBU, 0x40U, /* VCOMH deselect level. */
        0xA4U,        /* Display follows GDDRAM. */
        0xA6U,        /* Normal, non-inverted pixels. */
        0x2EU,        /* Scrolling disabled. */
        0xAFU         /* Display on. */
    };
    if ((address < 0x08U) || (address > 0x77U) || (budget == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    for (size_t index = 0U; index < sizeof(commands); ++index)
    {
        avr_status_t status = send_command(address, commands[index], budget);
        if (status != AVR_OK)
        {
            return status;
        }
    }
    return AVR_OK;
}

avr_status_t avr_ssd1306_write_page(uint8_t address, uint8_t page, const uint8_t *columns,
                                    uint32_t budget)
{
    if ((address < 0x08U) || (address > 0x77U) || (page > 7U) || (columns == NULL) ||
        (budget == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    const uint8_t page_command = (uint8_t)(0xB0U | page);
    avr_status_t status = send_command(address, page_command, budget);
    if (status == AVR_OK)
    {
        status = send_command(address, 0x00U, budget); /* Low column address. */
    }
    if (status == AVR_OK)
    {
        status = send_command(address, 0x10U, budget); /* High column address. */
    }
    if (status != AVR_OK)
    {
        return status;
    }
    uint8_t packet[129];
    packet[0] = 0x40U; /* Data control byte selects GDDRAM writes. */
    for (uint8_t column = 0U; column < 128U; ++column)
    {
        packet[(size_t)column + 1U] = columns[column];
    }
    return avr_twi_transfer(address, packet, (uint8_t)sizeof(packet), NULL, 0U, budget);
}

avr_status_t avr_ssd1306_clear(uint8_t address, uint32_t budget)
{
    if ((address < 0x08U) || (address > 0x77U) || (budget == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    uint8_t blank[128] = {0U};
    for (uint8_t page = 0U; page < 8U; ++page)
    {
        avr_status_t status = avr_ssd1306_write_page(address, page, blank, budget);
        if (status != AVR_OK)
        {
            return status;
        }
    }
    return AVR_OK;
}
