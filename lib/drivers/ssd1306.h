/**
 * @file ssd1306.h
 * @brief Small page-oriented SSD1306 I2C display interface.
 * @details Targets common 128 by 64 SSD1306 breakout modules over the AVR HAL TWI controller.
 * Stores one 128-byte page at a time rather than reserving a full 1024-byte framebuffer.
 * Verify the module address, supply and pull-up voltage before use.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_SSD1306_H
#define AVR_SSD1306_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Initialize a common 128 by 64 SSD1306 I2C module.
     * @param[in] address Unshifted 7-bit I2C address, normally 0x3C or 0x3D.
     * @param[in] budget Maximum unsuccessful polls per TWI state; not elapsed time.
     * @retval AVR_OK Initialization commands were acknowledged.
     * @retval AVR_ERROR_ARGUMENT Invalid address or zero polling budget.
     * @retval AVR_ERROR_NACK Module did not acknowledge a command.
     * @retval AVR_ERROR_TIMEOUT Bus operation exceeded its poll budget.
     * @retval AVR_ERROR_IO Bus was disabled or returned an unexpected status.
     * @retval AVR_ERROR_ARBITRATION Another controller won arbitration.
     * @details Requires avr_twi_init() first. Sends a fixed sequence for 128x64, page addressing,
     * internal charge pump and a common panel orientation. Breakout variants may need different
     * contrast or COM-pin settings. Foreground only; blocking but each bus state is bounded.
     */
    avr_status_t avr_ssd1306_init(uint8_t address, uint32_t budget);

    /**
     * @brief Replace one horizontal page of pixels.
     * @param[in] address Unshifted 7-bit address used during initialization.
     * @param[in] page Display page 0..7; each page is eight vertical pixels high.
     * @param[in] columns Caller-owned array of exactly 128 column bytes, least-significant bit at
     * the top row of each page. The data is transmitted synchronously and not retained.
     * @param[in] budget Maximum unsuccessful polls per TWI state; not elapsed time.
     * @retval AVR_OK Page data was acknowledged.
     * @retval AVR_ERROR_ARGUMENT Invalid address, page, data pointer or zero budget.
     * @retval AVR_ERROR_NACK Module did not acknowledge a command or data byte.
     * @retval AVR_ERROR_TIMEOUT Bus operation exceeded its poll budget.
     * @retval AVR_ERROR_IO Bus was disabled or returned an unexpected status.
     * @retval AVR_ERROR_ARBITRATION Another controller won arbitration.
     * @details Uses 129 bytes of stack for one control byte plus the caller's page. The display
     * retains the image in its own GDDRAM; the function does not keep a 1 KB MCU framebuffer.
     * Foreground only and blocking for the complete page transfer.
     */
    avr_status_t avr_ssd1306_write_page(uint8_t address, uint8_t page, const uint8_t *columns,
                                        uint32_t budget);

    /**
     * @brief Clear all eight display pages.
     * @param[in] address Unshifted 7-bit address used during initialization.
     * @param[in] budget Maximum unsuccessful polls per TWI state; not elapsed time.
     * @retval AVR_OK All eight pages were transmitted as zero.
     * @retval AVR_ERROR_ARGUMENT Invalid address or zero budget.
     * @retval AVR_ERROR_NACK Module did not acknowledge a command or data byte.
     * @retval AVR_ERROR_TIMEOUT Bus operation exceeded its poll budget.
     * @retval AVR_ERROR_IO Bus was disabled or returned an unexpected status.
     * @retval AVR_ERROR_ARBITRATION Another controller won arbitration.
     * @details Uses a 128-byte stack buffer and one page transfer at a time. A failure may leave
     * some pages cleared and later pages unchanged. Foreground only; does not retain a framebuffer.
     */
    avr_status_t avr_ssd1306_clear(uint8_t address, uint32_t budget);
#ifdef __cplusplus
}
#endif
#endif
