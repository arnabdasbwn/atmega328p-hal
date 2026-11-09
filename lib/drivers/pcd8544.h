/**
 * @file pcd8544.h
 * @brief Low-memory page writer for the PCD8544 84 by 48 display.
 * @details Uses the AVR HAL SPI controller and caller-supplied D7..D9 control pins. One 84-byte
 * page is provided by the caller, avoiding a full 504-byte framebuffer.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_PCD8544_H
#define AVR_DRIVER_PCD8544_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Number of pixel columns in one display RAM page. */
#define AVR_PCD8544_COLUMNS UINT8_C(84)
    /** @brief Number of eight-row pages in the 48-row display. */
#define AVR_PCD8544_PAGES UINT8_C(6)

    /** @brief Caller-owned display connection and contrast settings. */
    typedef struct
    {
        uint8_t chip_select_pin;  /**< Uno GPIO pin connected to LCD CS. */
        uint8_t data_command_pin; /**< Uno GPIO pin selecting instruction/data bytes. */
        uint8_t reset_pin;        /**< Uno GPIO pin connected to LCD reset. */
        uint8_t contrast;         /**< Contrast value in the range 0..127. */
        bool initialized;         /**< True after successful hardware initialization. */
    } avr_pcd8544_t;

    /**
     * @brief Reset and configure a PCD8544 display.
     * @param[out] display Caller-owned context; updated only after successful setup.
     * @param[in] chip_select_pin GPIO pin for active-low CS; must not be D10..D13.
     * @param[in] data_command_pin GPIO pin for D/C; must not be D10..D13 or CS.
     * @param[in] reset_pin GPIO pin for RESET; must not be D10..D13, CS or D/C.
     * @param[in] contrast Electronic contrast setting 0..127; panel voltage depends on the LCD.
     * @return AVR_OK after configuration, or argument/GPIO/SPI timeout status.
     * @details Initializes SPI mode 0 at 2 MHz (F_CPU/8), reserves D10..D13 through the SPI HAL,
     * then resets and configures the display. It does not enable global interrupts. Physical signal
     * voltage translation and module supply are board-level responsibilities.
     */
    avr_status_t avr_pcd8544_init(avr_pcd8544_t *display, uint8_t chip_select_pin,
                                  uint8_t data_command_pin, uint8_t reset_pin, uint8_t contrast);

    /**
     * @brief Replace one eight-pixel-high display page.
     * @param[in] display Initialized display context; the function does not modify it.
     * @param[in] page Page index 0..5, with page 0 at the top of the screen.
     * @param[in] pixels Caller-owned array of exactly 84 bytes; each byte stores one vertical
     * eight-pixel column, least-significant bit first as specified by the controller.
     * @return AVR_OK after all bytes transfer, argument error for invalid input, or SPI/GPIO error.
     * @details Blocking foreground operation. Does not retain the buffer or allocate a framebuffer.
     * CS is deasserted on every return path. After a transfer error, reinitialize the display
     * before retrying because the controller address may have advanced partially.
     */
    avr_status_t avr_pcd8544_write_page(const avr_pcd8544_t *display, uint8_t page,
                                        const uint8_t pixels[AVR_PCD8544_COLUMNS]);
#ifdef __cplusplus
}
#endif
#endif
