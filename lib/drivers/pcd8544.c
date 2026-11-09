/**
 * @file pcd8544.c
 * @brief SPI command and page operations for the PCD8544 display.
 * @details Uses the controller's 84-column by 6-page RAM map and sends bytes MSB first through the
 * shared AVR SPI HAL. Display commands and pixel data are kept separate by the D/C GPIO line.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "pcd8544.h"
#include "avr_hal/gpio.h"
#include "avr_hal/spi.h"
#include <util/delay.h>

/**
 * @brief Check whether a control pin can be used beside the SPI-owned Uno pins.
 * @param[in] pin Uno header pin number.
 * @return True for GPIO 0..19 except D10..D13.
 */
static bool avr_pcd8544_control_pin_valid(uint8_t pin)
{
    return (pin < 20U) && !((pin >= 10U) && (pin <= 13U));
}

/**
 * @brief Send one byte while chip select is active.
 * @param[in] value Command or display data byte.
 * @return Status from the bounded SPI transfer.
 * @details The caller must have selected the D/C mode and asserted CS.
 */
static avr_status_t avr_pcd8544_send_byte(uint8_t value)
{
    uint8_t discarded;
    return avr_spi_transfer(value, &discarded, AVR_POLL_BUDGET);
}

/**
 * @brief Send one instruction with a separate chip-select interval.
 * @param[in,out] display Initialized display context.
 * @param[in] command Controller instruction byte.
 * @return GPIO or bounded SPI transfer status.
 * @details Always attempts to deassert CS after the transfer.
 */
static avr_status_t avr_pcd8544_command(const avr_pcd8544_t *display, uint8_t command)
{
    avr_status_t status = avr_gpio_write(display->data_command_pin, false);
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->chip_select_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_send_byte(command);
    }
    if (avr_gpio_write(display->chip_select_pin, true) != AVR_OK)
    {
        return AVR_ERROR_IO;
    }
    return status;
}

avr_status_t avr_pcd8544_init(avr_pcd8544_t *display, uint8_t chip_select_pin,
                              uint8_t data_command_pin, uint8_t reset_pin, uint8_t contrast)
{
    avr_pcd8544_t candidate;
    avr_status_t status;
    if ((display == NULL) || !avr_pcd8544_control_pin_valid(chip_select_pin) ||
        !avr_pcd8544_control_pin_valid(data_command_pin) ||
        !avr_pcd8544_control_pin_valid(reset_pin) || (chip_select_pin == data_command_pin) ||
        (chip_select_pin == reset_pin) || (data_command_pin == reset_pin) || (contrast > 127U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_spi_init(0U, 8U);
    if (status != AVR_OK)
    {
        return status;
    }
    status = avr_gpio_mode(chip_select_pin, AVR_GPIO_OUTPUT);
    if (status == AVR_OK)
    {
        status = avr_gpio_mode(data_command_pin, AVR_GPIO_OUTPUT);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_mode(reset_pin, AVR_GPIO_OUTPUT);
    }
    if (status != AVR_OK)
    {
        return status;
    }
    candidate.chip_select_pin = chip_select_pin;
    candidate.data_command_pin = data_command_pin;
    candidate.reset_pin = reset_pin;
    candidate.contrast = contrast;
    candidate.initialized = false;
    status = avr_gpio_write(chip_select_pin, true);
    if (status == AVR_OK)
    {
        status = avr_gpio_write(data_command_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(reset_pin, false);
    }
    if (status != AVR_OK)
    {
        return status;
    }
    _delay_ms(10);
    status = avr_gpio_write(reset_pin, true);
    if (status != AVR_OK)
    {
        return status;
    }
    _delay_ms(10);
    status = avr_pcd8544_command(&candidate, UINT8_C(0x21));
    if (status == AVR_OK)
    {
        status = avr_pcd8544_command(&candidate, (uint8_t)(UINT8_C(0x80) | contrast));
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_command(&candidate, UINT8_C(0x14));
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_command(&candidate, UINT8_C(0x20));
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_command(&candidate, UINT8_C(0x0C));
    }
    if (status == AVR_OK)
    {
        candidate.initialized = true;
        *display = candidate;
    }
    return status;
}

avr_status_t avr_pcd8544_write_page(const avr_pcd8544_t *display, uint8_t page,
                                    const uint8_t pixels[AVR_PCD8544_COLUMNS])
{
    avr_status_t status;
    if ((display == NULL) || (pixels == NULL) || !display->initialized ||
        (page >= AVR_PCD8544_PAGES))
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_gpio_write(display->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->data_command_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_send_byte(UINT8_C(0x80));
    }
    if (status == AVR_OK)
    {
        status = avr_pcd8544_send_byte((uint8_t)(UINT8_C(0x40) | page));
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(display->data_command_pin, true);
    }
    for (uint8_t column = 0U; (column < AVR_PCD8544_COLUMNS) && (status == AVR_OK); column++)
    {
        status = avr_pcd8544_send_byte(pixels[column]);
    }
    if (avr_gpio_write(display->chip_select_pin, true) != AVR_OK)
    {
        return AVR_ERROR_IO;
    }
    return status;
}
