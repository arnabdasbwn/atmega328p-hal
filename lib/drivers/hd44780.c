/**
 * @file hd44780.c
 * @brief Four-bit, write-only HD44780 LCD interface.
 * @details Uses six caller-selected Uno GPIO pins and requires RW tied LOW. Sends a power-on
 * synchronization sequence and high-nibble-first data. Fixed delays avoid busy-flag reads; the
 * driver cannot detect an absent display.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "hd44780.h"
#include "avr_hal/gpio.h"
#include <util/delay.h>
/**
 * @brief Validate that six distinct supported GPIO pins describe the display bus.
 * @param[in] lcd Configuration pointer; NULL is rejected.
 * @return True for six distinct pins in 0..19; false otherwise.
 * @details No hardware access; does not verify that a physical LCD is connected.
 */
static bool valid(const avr_lcd_t *lcd)
{
    if (lcd == NULL)
    {
        return false;
    }
    uint8_t pins[6] = {lcd->rs,      lcd->enable,  lcd->data[0],
                       lcd->data[1], lcd->data[2], lcd->data[3]};
    for (uint8_t i = 0U; i < 6U; ++i)
    {
        if (pins[i] >= 20U)
        {
            return false;
        }
        for (uint8_t j = 0U; j < i; ++j)
        {
            if (pins[i] == pins[j])
            {
                return false;
            }
        }
    }
    return true;
}
/**
 * @brief Drive four data bits and pulse the enable strobe.
 * @param[in] lcd Previously validated display configuration.
 * @param[in] value Low four bits sent to DB4..DB7; upper bits ignored.
 * @details Holds enable high for at least one microsecond. Foreground only; no busy-flag read.
 */
static void nibble(const avr_lcd_t *lcd, uint8_t value)
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)avr_gpio_write(lcd->data[i], (value & (uint8_t)(1U << i)) != 0U);
    }
    (void)avr_gpio_write(lcd->enable, true);
    _delay_us(1);
    (void)avr_gpio_write(lcd->enable, false);
    _delay_us(1);
}
/**
 * @brief Send high/low nibbles and allow a normal instruction to finish.
 * @param[in] lcd Previously validated, initialized display configuration.
 * @param[in] value Eight-bit command or character code.
 * @param[in] data True selects character data; false selects a command.
 * @details Fixed 50 us delay follows the transfer. Clear/home require an additional delay.
 */
static void byte(const avr_lcd_t *lcd, uint8_t value, bool data)
{
    (void)avr_gpio_write(lcd->rs, data);
    nibble(lcd, (uint8_t)(value >> 4U));
    nibble(lcd, (uint8_t)(value & 15U));
    _delay_us(50);
}
/** @see hd44780.h for the authoritative API contract. */
avr_status_t avr_lcd_init(const avr_lcd_t *lcd)
{
    if (!valid(lcd))
    {
        return AVR_ERROR_ARGUMENT;
    }
    (void)avr_gpio_mode(lcd->rs, AVR_GPIO_OUTPUT);
    (void)avr_gpio_mode(lcd->enable, AVR_GPIO_OUTPUT);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)avr_gpio_mode(lcd->data[i], AVR_GPIO_OUTPUT);
    }
    _delay_ms(40);
    nibble(lcd, 3U);
    _delay_ms(5);
    nibble(lcd, 3U);
    _delay_us(150);
    nibble(lcd, 3U);
    nibble(lcd, 2U);
    byte(lcd, 0x28U, false);
    byte(lcd, 0x08U, false);
    byte(lcd, 0x01U, false);
    _delay_ms(2);
    byte(lcd, 0x06U, false);
    byte(lcd, 0x0CU, false);
    return AVR_OK;
}
/** @see hd44780.h for the authoritative API contract. */
avr_status_t avr_lcd_position(const avr_lcd_t *lcd, uint8_t row, uint8_t column)
{
    if (!valid(lcd) || (row > 1U) || (column > 15U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    byte(lcd, (uint8_t)(0x80U | (row ? 0x40U : 0U) | column), false);
    return AVR_OK;
}
/** @see hd44780.h for the authoritative API contract. */
avr_status_t avr_lcd_write(const avr_lcd_t *lcd, const char *text, uint8_t length)
{
    if (!valid(lcd) || (text == NULL) || (length > 32U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    for (uint8_t i = 0U; i < length; ++i)
    {
        byte(lcd, (uint8_t)text[i], true);
    }
    return AVR_OK;
}
