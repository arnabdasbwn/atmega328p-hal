/**
 * @file sd_card.c
 * @brief Bounded CMD17 reads from version 2 SDHC cards over AVR SPI.
 * @details This deliberately small driver provides block reads only; partition and filesystem
 * parsing live in fat16.c. All command, response, token and byte loops have finite caller-supplied
 * or protocol-defined bounds.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "drivers/sd_card.h"
#include "avr_hal/gpio.h"
#include "avr_hal/spi.h"

/** @brief SD SPI command-frame prefix. */
#define AVR_SD_COMMAND_PREFIX UINT8_C(0x40)
/** @brief SD data-start token for a single-block read. */
#define AVR_SD_DATA_TOKEN UINT8_C(0xFE)
/** @brief Maximum response-byte polls after one command frame. */
#define AVR_SD_R1_POLL_LIMIT UINT8_C(10)
/** @brief Number of initial clock bytes with card select inactive. */
#define AVR_SD_POWERUP_CLOCK_BYTES UINT8_C(10)

/**
 * @brief Exchange one SPI byte using the budget stored in card state.
 * @param[in] card Initialized or initializing card state.
 * @param[in] output Byte shifted to the card.
 * @param[out] input Non-null destination for the byte shifted from the card.
 * @return The first status returned by the AVR SPI HAL.
 */
static avr_status_t avr_sd_exchange(const avr_sd_card_t *card, uint8_t output, uint8_t *input)
{
    return avr_spi_transfer(output, input, card->spi_poll_budget);
}

/**
 * @brief Deselect the card and supply one trailing idle clock byte.
 * @param[in] card Card whose CS pin is released.
 * @retval AVR_OK CS was released and the trailing clock completed.
 * @retval AVR_ERROR_IO The GPIO operation failed.
 * @retval AVR_ERROR_TIMEOUT The trailing SPI byte did not complete within its bound.
 */
static avr_status_t avr_sd_deselect(const avr_sd_card_t *card)
{
    uint8_t ignored = UINT8_MAX;
    avr_status_t status = avr_gpio_write(card->chip_select_pin, true);
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, UINT8_MAX, &ignored);
    }
    return status;
}

/**
 * @brief Release the selected card while preserving an earlier operation error.
 * @param[in] card Card whose CS pin must be released.
 * @param[in] prior_status First status from the transaction.
 * @return The prior failure, or the release status when the operation had succeeded.
 */
static avr_status_t avr_sd_release_preserving(const avr_sd_card_t *card, avr_status_t prior_status)
{
    const avr_status_t release_status = avr_sd_deselect(card);
    return (prior_status == AVR_OK) ? release_status : prior_status;
}

/**
 * @brief Wait until the deselected card reports the SPI idle byte.
 * @param[in] card Initialized card state and response-attempt limit.
 * @retval AVR_OK The card returned 0xFF within the configured bound.
 * @retval AVR_ERROR_TIMEOUT The card remained busy for every permitted poll.
 * @retval AVR_ERROR_IO or AVR_ERROR_BUSY An SPI transfer failed.
 * @details CS must be inactive while this helper clocks the bus.
 */
static avr_status_t avr_sd_wait_ready(const avr_sd_card_t *card)
{
    for (uint16_t attempt = 0U; attempt < card->response_poll_limit; attempt++)
    {
        uint8_t received = 0U;
        const avr_status_t status = avr_sd_exchange(card, UINT8_MAX, &received);
        if (status != AVR_OK)
        {
            return status;
        }
        if (received == UINT8_MAX)
        {
            return AVR_OK;
        }
    }
    return AVR_ERROR_TIMEOUT;
}

/**
 * @brief Send one six-byte SD command and collect its R1 response.
 * @param[in] card Card state and SPI bound.
 * @param[in] command Command number in the range 0..63.
 * @param[in] argument Big-endian SD command argument.
 * @param[out] response Destination for R1; written only on success.
 * @retval AVR_OK A response byte with its high bit clear was received.
 * @retval AVR_ERROR_ARGUMENT Invalid card, command, or response pointer.
 * @retval AVR_ERROR_TIMEOUT The bounded response poll expired.
 * @retval AVR_ERROR_IO A GPIO/SPI transfer failed.
 * @details Leaves CS asserted on success so commands with trailing response bytes can consume
 * them before deselection. Releases CS on every failure path.
 */
static avr_status_t avr_sd_command(const avr_sd_card_t *card, uint8_t command, uint32_t argument,
                                   uint8_t *response)
{
    uint8_t received = UINT8_MAX;
    const uint8_t crc = (command == 0U)   ? UINT8_C(0x95)
                        : (command == 8U) ? UINT8_C(0x87)
                                          : UINT8_C(0x01);
    avr_status_t status;
    if ((card == NULL) || (response == NULL) || (command > 63U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_gpio_write(card->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, (uint8_t)(AVR_SD_COMMAND_PREFIX | command), &received);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, (uint8_t)(argument >> 24U), &received);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, (uint8_t)(argument >> 16U), &received);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, (uint8_t)(argument >> 8U), &received);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, (uint8_t)argument, &received);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_exchange(card, crc, &received);
    }
    for (uint8_t attempt = 0U; (status == AVR_OK) && (attempt < AVR_SD_R1_POLL_LIMIT); attempt++)
    {
        status = avr_sd_exchange(card, UINT8_MAX, &received);
        if ((status == AVR_OK) && ((received & UINT8_C(0x80)) == 0U))
        {
            *response = received;
            return AVR_OK;
        }
    }
    (void)avr_sd_deselect(card);
    return (status == AVR_OK) ? AVR_ERROR_TIMEOUT : status;
}

/**
 * @brief Read a fixed byte count while CS remains asserted.
 * @param[in] card Selected card and SPI budget.
 * @param[out] bytes Destination buffer.
 * @param[in] length Number of bytes to read.
 * @retval AVR_OK All requested bytes were clocked.
 * @retval AVR_ERROR_ARGUMENT Invalid pointer for a nonzero length.
 * @retval AVR_ERROR_IO or AVR_ERROR_TIMEOUT The first SPI transfer error.
 */
static avr_status_t avr_sd_read_bytes(const avr_sd_card_t *card, uint8_t *bytes, uint16_t length)
{
    if ((bytes == NULL) && (length != 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    for (uint16_t index = 0U; index < length; index++)
    {
        const avr_status_t status = avr_sd_exchange(card, UINT8_MAX, &bytes[index]);
        if (status != AVR_OK)
        {
            return status;
        }
    }
    return AVR_OK;
}

/**
 * @brief Run CMD55 followed by ACMD41 and inspect the R1 status.
 * @param[in] card Card being initialized.
 * @param[out] response ACMD41 response.
 * @retval AVR_OK Both command frames completed.
 * @retval AVR_ERROR_TIMEOUT A bounded command response or SPI transfer expired.
 * @retval AVR_ERROR_IO The card returned an unexpected command state or the bus failed.
 */
static avr_status_t avr_sd_request_ready(const avr_sd_card_t *card, uint8_t *response)
{
    uint8_t r1 = UINT8_MAX;
    avr_status_t status = avr_sd_command(card, 55U, 0U, &r1);
    if (status == AVR_OK)
    {
        status = avr_sd_deselect(card);
    }
    if (status == AVR_OK)
    {
        if ((r1 != 0U) && (r1 != 1U))
        {
            return AVR_ERROR_IO;
        }
        status = avr_sd_command(card, 41U, UINT32_C(0x40000000), response);
    }
    return status;
}

avr_status_t avr_sd_card_init(avr_sd_card_t *card, uint8_t chip_select_pin,
                              uint32_t spi_poll_budget, uint16_t response_poll_limit)
{
    uint8_t r1 = UINT8_MAX;
    avr_status_t status;
    if ((card == NULL) || (chip_select_pin >= 20U) ||
        ((chip_select_pin >= 10U) && (chip_select_pin <= 13U)) || (spi_poll_budget == 0UL) ||
        (response_poll_limit == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    card->chip_select_pin = chip_select_pin;
    card->spi_poll_budget = spi_poll_budget;
    card->response_poll_limit = response_poll_limit;
    card->initialized = false;
    status = avr_gpio_mode(chip_select_pin, AVR_GPIO_OUTPUT);
    if (status == AVR_OK)
    {
        status = avr_gpio_write(chip_select_pin, true);
    }
    if (status == AVR_OK)
    {
        status = avr_spi_init(0U, UINT16_C(128));
    }
    for (uint8_t index = 0U; (status == AVR_OK) && (index < AVR_SD_POWERUP_CLOCK_BYTES); index++)
    {
        uint8_t ignored = UINT8_MAX;
        status = avr_sd_exchange(card, UINT8_MAX, &ignored);
    }
    if (status == AVR_OK)
    {
        status = avr_sd_command(card, 0U, 0U, &r1);
        if ((status == AVR_OK) && (r1 != 1U))
        {
            status = AVR_ERROR_UNSUPPORTED;
        }
    }
    status = avr_sd_release_preserving(card, status);
    if (status == AVR_OK)
    {
        status = avr_sd_command(card, 8U, UINT32_C(0x000001AA), &r1);
        if ((status == AVR_OK) && (r1 == 1U))
        {
            uint8_t r7[4] = {0U, 0U, 0U, 0U};
            status = avr_sd_read_bytes(card, r7, (uint16_t)sizeof(r7));
            if ((status == AVR_OK) && ((r7[2] != 1U) || (r7[3] != UINT8_C(0xAA))))
            {
                status = AVR_ERROR_UNSUPPORTED;
            }
        }
        else if (status == AVR_OK)
        {
            status = AVR_ERROR_UNSUPPORTED;
        }
    }
    status = avr_sd_release_preserving(card, status);
    if (status == AVR_OK)
    {
        bool ready = false;
        for (uint16_t attempt = 0U; (status == AVR_OK) && !ready; attempt++)
        {
            status = avr_sd_request_ready(card, &r1);
            const avr_status_t release_status = avr_sd_deselect(card);
            if ((status == AVR_OK) && (release_status != AVR_OK))
            {
                status = release_status;
            }
            if (status == AVR_OK)
            {
                ready = (r1 == 0U);
                if (!ready && (r1 != 1U))
                {
                    status = AVR_ERROR_IO;
                }
                else if (!ready && ((uint16_t)(attempt + 1U) >= response_poll_limit))
                {
                    status = AVR_ERROR_TIMEOUT;
                }
            }
        }
        if ((status == AVR_OK) && !ready)
        {
            status = AVR_ERROR_TIMEOUT;
        }
    }
    if (status == AVR_OK)
    {
        status = avr_sd_command(card, 58U, 0U, &r1);
        if ((status == AVR_OK) && (r1 == 0U))
        {
            uint8_t ocr[4] = {0U, 0U, 0U, 0U};
            status = avr_sd_read_bytes(card, ocr, (uint16_t)sizeof(ocr));
            if ((status == AVR_OK) && ((ocr[0] & UINT8_C(0x40)) == 0U))
            {
                status = AVR_ERROR_UNSUPPORTED;
            }
        }
        else if (status == AVR_OK)
        {
            status = AVR_ERROR_IO;
        }
    }
    status = avr_sd_release_preserving(card, status);
    if (status == AVR_OK)
    {
        card->initialized = true;
    }
    return status;
}

avr_status_t avr_sd_card_read_sector(const avr_sd_card_t *card, uint32_t logical_block_address,
                                     uint8_t sector[AVR_SD_SECTOR_SIZE])
{
    uint8_t r1 = UINT8_MAX;
    uint8_t token = UINT8_MAX;
    avr_status_t status;
    if ((card == NULL) || (sector == NULL) || !card->initialized)
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_sd_wait_ready(card);
    if (status == AVR_OK)
    {
        status = avr_sd_command(card, 17U, logical_block_address, &r1);
    }
    if ((status == AVR_OK) && (r1 != 0U))
    {
        status = AVR_ERROR_IO;
    }
    for (uint16_t attempt = 0U;
         (status == AVR_OK) && (attempt < card->response_poll_limit) && (token == UINT8_MAX);
         attempt++)
    {
        status = avr_sd_exchange(card, UINT8_MAX, &token);
    }
    if ((status == AVR_OK) && (token != AVR_SD_DATA_TOKEN))
    {
        status = (token == UINT8_MAX) ? AVR_ERROR_TIMEOUT : AVR_ERROR_IO;
    }
    if (status == AVR_OK)
    {
        status = avr_sd_read_bytes(card, sector, AVR_SD_SECTOR_SIZE);
    }
    if (status == AVR_OK)
    {
        uint8_t crc[2] = {0U, 0U};
        status = avr_sd_read_bytes(card, crc, (uint16_t)sizeof(crc));
    }
    const avr_status_t release_status = avr_sd_deselect(card);
    if ((status == AVR_OK) && (release_status != AVR_OK))
    {
        status = release_status;
    }
    return status;
}
