/**
 * @file spi.h
 * @brief MSB-first SPI controller transfers.
 * @details Owns D10/SS, D11/COPI, D12/CIPO and D13/SCK. Peripheral chip select remains the
 * application responsibility. Controller mode requires SS configured as an output. Transfers are
 * bounded and nonreentrant.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_HAL_SPI_H
#define AVR_HAL_SPI_H
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Configure the hardware SPI controller, MSB first.
     * @param[in] mode SPI mode 0..3 (CPOL/CPHA combination).
     * @param[in] divider CPU clock divisor: 2, 4, 8, 16, 32, 64 or 128.
     * @retval AVR_OK SPI configured.
     * @retval AVR_ERROR_ARGUMENT Unsupported mode/divider; registers unchanged.
     * @details Owns D10/SS output HIGH, D11/COPI output, D12/CIPO input and D13/SCK output. Sets
     * clock idle level before starting SPI. Disables SPI interrupts; no global interrupt changes.
     * Configure peripheral chip-select separately.
     */
    avr_status_t avr_spi_init(uint8_t mode, uint16_t divider);
    /**
     * @brief Exchange one byte over SPI.
     * @param[in] transmit Byte shifted out MSB first.
     * @param[out] receive Non-null destination; unchanged on errors.
     * @param[in] budget Maximum unsuccessful completion polls, not a time unit.
     * @retval AVR_OK Received byte stored.
     * @retval AVR_ERROR_ARGUMENT Null output pointer.
     * @retval AVR_ERROR_IO SPI disabled or no longer controller.
     * @retval AVR_ERROR_TIMEOUT Transfer did not complete within budget.
     * @retval AVR_ERROR_BUSY Write collision detected.
     * @details Foreground, single caller after initialization; no reentrant access. Caller
     * asserts/deasserts chip-select. On timeout the transfer may still be active: deassert
     * chip-select and reinitialize before retrying. Reads status then data to clear
     * completion/collision flags.
     */
    avr_status_t avr_spi_transfer(uint8_t transmit, uint8_t *receive, uint32_t budget);
#ifdef __cplusplus
}
#endif
#endif
