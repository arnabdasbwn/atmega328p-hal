/**
 * @file st7735.c
 * @brief Bounded SPI command and RGB565 rectangle writer for ST7735R panels.
 * @details Uses a fixed SPI completion budget and no pixel buffer. Initialization timing and
 * register values follow a common ST7735R profile; panel variants may require different values.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "drivers/st7735.h"
#include "avr_hal/gpio.h"
#include "avr_hal/spi.h"
#include <util/delay.h>

/** @brief Maximum failed SPI completion polls per transmitted byte. */
#define AVR_ST7735_SPI_BUDGET UINT32_C(100000)

/** @brief ST7735R software reset command. */
#define AVR_ST7735_CMD_SWRESET UINT8_C(0x01)
/** @brief ST7735R sleep-out command. */
#define AVR_ST7735_CMD_SLPOUT UINT8_C(0x11)
/** @brief ST7735R memory-access control command. */
#define AVR_ST7735_CMD_MADCTL UINT8_C(0x36)
/** @brief ST7735R pixel-format command. */
#define AVR_ST7735_CMD_COLMOD UINT8_C(0x3A)
/** @brief ST7735R column-address window command. */
#define AVR_ST7735_CMD_CASET UINT8_C(0x2A)
/** @brief ST7735R row-address window command. */
#define AVR_ST7735_CMD_RASET UINT8_C(0x2B)
/** @brief ST7735R memory-write command. */
#define AVR_ST7735_CMD_RAMWR UINT8_C(0x2C)
/** @brief ST7735R normal-display mode command. */
#define AVR_ST7735_CMD_NORON UINT8_C(0x13)
/** @brief ST7735R display-on command. */
#define AVR_ST7735_CMD_DISPON UINT8_C(0x29)

/**
 * @brief Exchange one byte while the caller owns chip select.
 * @param[in] byte Byte sent MSB first.
 * @return Status from the AVR HAL SPI transfer.
 * @details The received byte is intentionally discarded because ST7735 writes do not need MISO.
 */
static avr_status_t avr_st7735_write_byte(uint8_t byte)
{
    uint8_t ignored = 0U;
    return avr_spi_transfer(byte, &ignored, AVR_ST7735_SPI_BUDGET);
}

/**
 * @brief Send one panel command followed by its optional data bytes.
 * @param[in] display Configured panel pins.
 * @param[in] command Command register value.
 * @param[in] data Optional command data; may be NULL only for zero length.
 * @param[in] length Number of data bytes.
 * @return AVR_OK or the first GPIO/SPI error.
 * @details CS is asserted for the complete command transaction and released on every exit.
 */
static avr_status_t avr_st7735_command(const avr_st7735_t *display, uint8_t command,
                                       const uint8_t *data, uint8_t length)
{
    avr_status_t status = avr_gpio_write(display->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->data_command_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_st7735_write_byte(command);
    }
    if ((status == AVR_OK) && (length > 0U))
    {
        status = avr_gpio_write(display->data_command_pin, true);
        for (uint8_t index = 0U; (index < length) && (status == AVR_OK); index++)
        {
            status = avr_st7735_write_byte(data[index]);
        }
    }
    if (avr_gpio_write(display->chip_select_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    return status;
}

/**
 * @brief Validate panel pins, dimensions and address-window bounds.
 * @param[in] cs Chip-select pin.
 * @param[in] dc Data/command pin.
 * @param[in] reset Reset pin.
 * @param[in] width Visible width.
 * @param[in] height Visible height.
 * @param[in] x_offset Visible start column.
 * @param[in] y_offset Visible start row.
 * @return True only if values fit the common ST7735 panel address space.
 */
static bool avr_st7735_arguments_valid(uint8_t cs, uint8_t dc, uint8_t reset, uint16_t width,
                                       uint16_t height, uint16_t x_offset, uint16_t y_offset)
{
    return ((cs < 20U) && (dc < 20U) && (reset < 20U) && (cs != dc) && (cs != reset) &&
            (dc != reset) && ((cs < 10U) || (cs > 13U)) && ((dc < 10U) || (dc > 13U)) &&
            ((reset < 10U) || (reset > 13U)) && (width > 0U) && (height > 0U) && (width <= 132U) &&
            (height <= 162U) && (x_offset <= (132U - width)) && (y_offset <= (162U - height)));
}

/**
 * @brief Initialize a common ST7735R panel for 16-bit RGB565 transfers.
 * @details Configures SPI mode 0 at F_CPU/4, applies software/hardware reset and the basic
 * ST7735R power/frame/color profile, then enables normal display output. Gamma and panel-specific
 * vendor settings are omitted so a particular breakout may require its manufacturer's sequence.
 * @see st7735.h for argument, return, buffer and board-level contracts.
 */
avr_status_t avr_st7735_init(avr_st7735_t *display, uint8_t chip_select_pin,
                             uint8_t data_command_pin, uint8_t reset_pin, uint16_t width,
                             uint16_t height, uint16_t x_offset, uint16_t y_offset, bool bgr_order)
{
    if ((display == NULL) ||
        !avr_st7735_arguments_valid(chip_select_pin, data_command_pin, reset_pin, width, height,
                                    x_offset, y_offset))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_st7735_t candidate = {chip_select_pin, data_command_pin, reset_pin, width, height,
                              x_offset,        y_offset,         bgr_order, false};
    avr_status_t status = avr_gpio_mode(chip_select_pin, AVR_GPIO_OUTPUT);
    if (status == AVR_OK)
    {
        status = avr_gpio_mode(data_command_pin, AVR_GPIO_OUTPUT);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_mode(reset_pin, AVR_GPIO_OUTPUT);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(chip_select_pin, true);
    }
    if (status == AVR_OK)
    {
        status = avr_spi_init(0U, 4U);
    }
    if (status != AVR_OK)
    {
        return status;
    }
    status = avr_gpio_write(reset_pin, false);
    if (status == AVR_OK)
    {
        _delay_ms(10);
        status = avr_gpio_write(reset_pin, true);
    }
    if (status != AVR_OK)
    {
        return status;
    }
    _delay_ms(120);

    status = avr_st7735_command(&candidate, AVR_ST7735_CMD_SWRESET, NULL, 0U);
    if (status == AVR_OK)
    {
        _delay_ms(150);
        status = avr_st7735_command(&candidate, AVR_ST7735_CMD_SLPOUT, NULL, 0U);
    }
    if (status == AVR_OK)
    {
        _delay_ms(120);
        const uint8_t frame_rate[] = {0x01U, 0x2CU, 0x2DU};
        status =
            avr_st7735_command(&candidate, UINT8_C(0xB1), frame_rate, (uint8_t)sizeof(frame_rate));
    }
    if (status == AVR_OK)
    {
        const uint8_t inversion[] = {0x07U};
        status = avr_st7735_command(&candidate, UINT8_C(0xB4), inversion, 1U);
    }
    if (status == AVR_OK)
    {
        const uint8_t power1[] = {0xA2U, 0x02U, 0x84U};
        status = avr_st7735_command(&candidate, UINT8_C(0xC0), power1, (uint8_t)sizeof(power1));
    }
    if (status == AVR_OK)
    {
        const uint8_t power2[] = {0xC5U};
        status = avr_st7735_command(&candidate, UINT8_C(0xC1), power2, 1U);
    }
    if (status == AVR_OK)
    {
        const uint8_t power3[] = {0x0AU, 0x00U};
        status = avr_st7735_command(&candidate, UINT8_C(0xC2), power3, (uint8_t)sizeof(power3));
    }
    if (status == AVR_OK)
    {
        const uint8_t power4[] = {0x8AU, 0x2AU};
        status = avr_st7735_command(&candidate, UINT8_C(0xC3), power4, (uint8_t)sizeof(power4));
    }
    if (status == AVR_OK)
    {
        const uint8_t power5[] = {0x8AU, 0xEEU};
        status = avr_st7735_command(&candidate, UINT8_C(0xC4), power5, (uint8_t)sizeof(power5));
    }
    if (status == AVR_OK)
    {
        const uint8_t vcom[] = {0x0EU};
        status = avr_st7735_command(&candidate, UINT8_C(0xC5), vcom, 1U);
    }
    if (status == AVR_OK)
    {
        const uint8_t pixel_format[] = {0x05U};
        status = avr_st7735_command(&candidate, AVR_ST7735_CMD_COLMOD, pixel_format, 1U);
    }
    if (status == AVR_OK)
    {
        const uint8_t memory_access[] = {bgr_order ? 0xC8U : 0xC0U};
        status = avr_st7735_command(&candidate, AVR_ST7735_CMD_MADCTL, memory_access, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_st7735_command(&candidate, AVR_ST7735_CMD_NORON, NULL, 0U);
    }
    if (status == AVR_OK)
    {
        _delay_ms(10);
        status = avr_st7735_command(&candidate, AVR_ST7735_CMD_DISPON, NULL, 0U);
    }
    if (status == AVR_OK)
    {
        _delay_ms(100);
        candidate.initialized = true;
        *display = candidate;
    }
    return status;
}

/**
 * @brief Set a 16-bit controller address boundary.
 * @param[in] display Configured panel.
 * @param[in] command CASET or RASET.
 * @param[in] first First inclusive address.
 * @param[in] last Last inclusive address.
 * @return AVR_OK or the first GPIO/SPI error.
 */
static avr_status_t avr_st7735_set_window_axis(const avr_st7735_t *display, uint8_t command,
                                               uint16_t first, uint16_t last)
{
    const uint8_t coordinates[] = {(uint8_t)(first >> 8U), (uint8_t)first, (uint8_t)(last >> 8U),
                                   (uint8_t)last};
    return avr_st7735_command(display, command, coordinates, (uint8_t)sizeof(coordinates));
}

/**
 * @brief Stream a solid RGB565 rectangle into the panel's display RAM.
 * @return AVR_OK or an argument/GPIO/SPI error.
 * @details Coordinates and extents are validated before the first display command. The routine
 * streams the high color byte then low byte per pixel; it never stores a full frame or line buffer.
 * @see st7735.h for parameter, return and blocking-behaviour contracts.
 */
avr_status_t avr_st7735_fill_rect(const avr_st7735_t *display, uint16_t x, uint16_t y,
                                  uint16_t width, uint16_t height, uint16_t color)
{
    if ((display == NULL) || !display->initialized || (width == 0U) || (height == 0U) ||
        (x >= display->width) || (y >= display->height) || (width > (display->width - x)) ||
        (height > (display->height - y)))
    {
        return AVR_ERROR_ARGUMENT;
    }
    const uint16_t x_start = (uint16_t)(x + display->x_offset);
    const uint16_t y_start = (uint16_t)(y + display->y_offset);
    avr_status_t status = avr_st7735_set_window_axis(display, AVR_ST7735_CMD_CASET, x_start,
                                                     (uint16_t)(x_start + width - 1U));
    if (status == AVR_OK)
    {
        status = avr_st7735_set_window_axis(display, AVR_ST7735_CMD_RASET, y_start,
                                            (uint16_t)(y_start + height - 1U));
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->chip_select_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->data_command_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_st7735_write_byte(AVR_ST7735_CMD_RAMWR);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->data_command_pin, true);
    }
    const uint8_t high_byte = (uint8_t)(color >> 8U);
    const uint8_t low_byte = (uint8_t)color;
    const uint32_t pixel_count = (uint32_t)width * (uint32_t)height;
    for (uint32_t pixel = 0U; (pixel < pixel_count) && (status == AVR_OK); pixel++)
    {
        status = avr_st7735_write_byte(high_byte);
        if (status == AVR_OK)
        {
            status = avr_st7735_write_byte(low_byte);
        }
    }
    if (avr_gpio_write(display->chip_select_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    return status;
}
