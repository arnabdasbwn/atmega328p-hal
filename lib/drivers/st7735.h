/**
 * @file st7735.h
 * @brief Low-memory solid-colour drawing for a common ST7735R SPI panel.
 * @details Streams RGB565 pixels directly over SPI and stores no framebuffer. Panel dimensions,
 * address offsets and RGB/BGR order belong to the exact breakout and must be confirmed.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_ST7735_H
#define AVR_DRIVER_ST7735_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief RGB565 red component for simple display tests. */
#define AVR_ST7735_RED UINT16_C(0xF800)
    /** @brief RGB565 green component for simple display tests. */
#define AVR_ST7735_GREEN UINT16_C(0x07E0)
    /** @brief RGB565 blue component for simple display tests. */
#define AVR_ST7735_BLUE UINT16_C(0x001F)
    /** @brief RGB565 black component for simple display tests. */
#define AVR_ST7735_BLACK UINT16_C(0x0000)

    /** @brief Caller-owned panel dimensions, offsets and board control pins. */
    typedef struct
    {
        uint8_t chip_select_pin;  /**< Uno GPIO connected to active-low panel CS. */
        uint8_t data_command_pin; /**< Uno GPIO connected to panel D/C. */
        uint8_t reset_pin;        /**< Uno GPIO connected to panel RESET. */
        uint16_t width;           /**< Visible panel columns, for example 128. */
        uint16_t height;          /**< Visible panel rows, for example 160. */
        uint16_t x_offset;        /**< Panel-RAM start column for this panel profile. */
        uint16_t y_offset;        /**< Panel-RAM start row for this panel profile. */
        bool bgr_order;           /**< True when this panel profile requires BGR ordering. */
        bool initialized;         /**< True after successful initialization. */
    } avr_st7735_t;

    /**
     * @brief Reset and configure a common ST7735R RGB565 SPI panel.
     * @param[out] display Caller-owned display settings; committed only after setup succeeds.
     * @param[in] chip_select_pin Uno GPIO for active-low CS; distinct from control pins and
     * D10..D13.
     * @param[in] data_command_pin Uno GPIO selecting command or pixel data.
     * @param[in] reset_pin Uno GPIO connected to panel RESET.
     * @param[in] width Visible width in pixels; range 1..132.
     * @param[in] height Visible height in pixels; range 1..162.
     * @param[in] x_offset First visible panel-RAM column for the exact panel profile.
     * @param[in] y_offset First visible panel-RAM row for the exact panel profile.
     * @param[in] bgr_order True when the panel profile expects BGR instead of RGB order.
     * @return AVR_OK after setup or an argument/GPIO/SPI error.
     * @details Configures SPI mode 0 at F_CPU/4 and uses fixed reset/sleep-out delays. This is a
     * teaching profile for common ST7735R panels, not a universal initialization sequence. Level
     * translation and safe power are board responsibilities. Does not enable global interrupts.
     */
    avr_status_t avr_st7735_init(avr_st7735_t *display, uint8_t chip_select_pin,
                                 uint8_t data_command_pin, uint8_t reset_pin, uint16_t width,
                                 uint16_t height, uint16_t x_offset, uint16_t y_offset,
                                 bool bgr_order);

    /**
     * @brief Fill a rectangle with one RGB565 colour without allocating a framebuffer.
     * @param[in] display Initialized caller-owned panel configuration.
     * @param[in] x Left coordinate within the visible display.
     * @param[in] y Top coordinate within the visible display.
     * @param[in] width Rectangle width; rectangle must fit the configured display.
     * @param[in] height Rectangle height; rectangle must fit the configured display.
     * @param[in] color RGB565 value, most-significant byte transmitted first.
     * @return AVR_OK after every pixel is sent, or an argument/GPIO/SPI error.
     * @details Blocking foreground operation. Sends two bytes per pixel over SPI; the caller owns
     * the display until completion. The method does not retain data or allocate dynamic memory.
     * CS is deasserted on every return path. After a transfer failure, reinitialize before retry.
     */
    avr_status_t avr_st7735_fill_rect(const avr_st7735_t *display, uint16_t x, uint16_t y,
                                      uint16_t width, uint16_t height, uint16_t color);
#ifdef __cplusplus
}
#endif
#endif
