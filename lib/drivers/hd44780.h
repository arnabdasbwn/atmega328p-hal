/**
 * @file hd44780.h
 * @brief Four-bit, write-only HD44780 LCD interface.
 * @details Uses six caller-selected Uno GPIO pins and requires RW tied LOW. Sends a power-on
 * synchronization sequence and high-nibble-first data. Fixed delays avoid busy-flag reads; the
 * driver cannot detect an absent display.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HD44780_H
#define AVR_HD44780_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Six distinct Uno GPIO pins used by a four-bit, write-only LCD bus. */
    typedef struct
    {
        uint8_t rs;      /**< Register-select pin: LOW command, HIGH character data. */
        uint8_t enable;  /**< Enable strobe pin; falling edge latches each nibble. */
        uint8_t data[4]; /**< Uno pins connected in order to LCD DB4, DB5, DB6, DB7. */
    } avr_lcd_t;
    /**
     * @brief Initialize a parallel HD44780-compatible 16x2 LCD.
     * @param[in] lcd Non-null pin configuration; six distinct Uno pins in 0..19. Caller retains
     * ownership.
     * @retval AVR_OK Initialization sequence sent.
     * @retval AVR_ERROR_ARGUMENT Null, duplicate or invalid pins; no pins changed.
     * @details Owns the configured GPIO, uses four-bit bus and fixed delays with RW tied LOW.
     * Blocks about 50 ms; foreground only. Does not read busy flag or detect a missing display.
     * Configure supply, contrast and backlight per the actual module.
     */
    avr_status_t avr_lcd_init(const avr_lcd_t *lcd);
    /**
     * @brief Set the DDRAM address for a 16x2 cursor position.
     * @param[in] lcd Valid initialized display configuration; caller-owned.
     * @param[in] row Zero-based row, 0 or 1.
     * @param[in] column Zero-based column, 0..15.
     * @retval AVR_OK Command sent.
     * @retval AVR_ERROR_ARGUMENT Invalid configuration or coordinates.
     * @details Foreground only; fixed approximately 50 us command delay. Does not read back the
     * display. Row bases are 0x00 and 0x40 for this module geometry.
     */
    avr_status_t avr_lcd_position(const avr_lcd_t *lcd, uint8_t row, uint8_t column);
    /**
     * @brief Write a bounded sequence of LCD character codes.
     * @param[in] lcd Valid initialized display configuration.
     * @param[in] text Non-null buffer with at least length bytes; no retained pointer.
     * @param[in] length Character count, 0..32. No automatic newline/wrapping handling.
     * @retval AVR_OK Bytes sent.
     * @retval AVR_ERROR_ARGUMENT Invalid pins, null text or length above 32.
     * @details Foreground only, fixed delay per byte. Uses the display character ROM, not UTF-8
     * decoding. Address transitions follow HD44780 DDRAM; position each visible row explicitly.
     */
    avr_status_t avr_lcd_write(const avr_lcd_t *lcd, const char *text, uint8_t length);
#ifdef __cplusplus
}
#endif
#endif
