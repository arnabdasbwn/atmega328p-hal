/**
 * @file sd_card.h
 * @brief Read-only SPI access to version 2 SDHC cards.
 * @details The driver supports 512-byte block addressing and leaves the SD filesystem layer to
 * the caller. CRC checking is disabled after the required CMD0/CMD8 command CRC bytes. A failed
 * sector read may have written a prefix of the requested sector buffer.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_SD_CARD_H
#define AVR_SD_CARD_H

#include "avr_hal/common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief The data block size used by SDHC cards and the FAT16 reader. */
#define AVR_SD_SECTOR_SIZE UINT16_C(512)

    /**
     * @brief Caller-owned state for one SPI SDHC card.
     * @details One instance represents one card and must not be used concurrently or from an ISR.
     */
    typedef struct
    {
        /** @brief Uno GPIO connected to the card's active-low CS input. */
        uint8_t chip_select_pin;
        /** @brief Maximum unsuccessful AVR SPI completion polls for one byte. */
        uint32_t spi_poll_budget;
        /** @brief Finite attempts for card-ready and data-token responses; this is not
         * milliseconds. */
        uint16_t response_poll_limit;
        /** @brief True after a version 2 SDHC card completed the supported initialisation sequence.
         */
        bool initialized;
    } avr_sd_card_t;

    /**
     * @brief Initialise an SPI card using the bounded version 2 SDHC start-up sequence.
     * @param[out] card Caller-owned state, unchanged when arguments are invalid.
     * @param[in] chip_select_pin Uno GPIO below 20 and outside the hardware SPI pins D10..D13.
     * @param[in] spi_poll_budget Nonzero AVR SPI completion poll limit for each byte.
     * @param[in] response_poll_limit Nonzero response and ACMD41 attempt limit.
     * @retval AVR_OK Version 2 SDHC card responded and reports block addressing.
     * @retval AVR_ERROR_ARGUMENT Invalid state pointer, pin, or zero polling bound.
     * @retval AVR_ERROR_TIMEOUT A bounded SPI, response, or ready wait expired.
     * @retval AVR_ERROR_BUSY The SPI HAL detected a write collision.
     * @retval AVR_ERROR_UNSUPPORTED Card is not the supported version 2 SDHC type.
     * @retval AVR_ERROR_IO GPIO, SPI, or card response failed.
     * @details Configures SPI mode 0 at F_CPU/128 (125 kHz on the 16 MHz Uno), D10..D13 ownership,
     * and the requested CS output. Does not enable global interrupts or write card data.
     */
    avr_status_t avr_sd_card_init(avr_sd_card_t *card, uint8_t chip_select_pin,
                                  uint32_t spi_poll_budget, uint16_t response_poll_limit);

    /**
     * @brief Read one 512-byte SDHC block by logical block address.
     * @param[in] card Initialized card state; foreground-only exclusive access is required.
     * @param[in] logical_block_address SDHC block address, not a byte address.
     * @param[out] sector Caller-owned buffer of at least AVR_SD_SECTOR_SIZE bytes; a prefix may be
     * written before an error is returned.
     * @retval AVR_OK All 512 bytes and both transmitted CRC bytes were clocked successfully.
     * @retval AVR_ERROR_ARGUMENT Null, uninitialized, or invalid card state/buffer.
     * @retval AVR_ERROR_TIMEOUT Card-ready, data-token, or SPI transfer bound expired.
     * @retval AVR_ERROR_BUSY The SPI HAL detected a write collision.
     * @retval AVR_ERROR_IO Card rejected the command or SPI/GPIO access failed.
     * @details Uses CMD17 and finite polling. The driver does not verify the data CRC, infer card
     * capacity, mount a partition, or own a 512-byte buffer. Recover from an SPI timeout by
     * deasserting CS and reinitializing the card before another transaction.
     */
    avr_status_t avr_sd_card_read_sector(const avr_sd_card_t *card, uint32_t logical_block_address,
                                         uint8_t sector[AVR_SD_SECTOR_SIZE]);

#ifdef __cplusplus
}
#endif
#endif
