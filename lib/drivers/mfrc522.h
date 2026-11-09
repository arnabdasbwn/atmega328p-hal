/**
 * @file mfrc522.h
 * @brief Bounded SPI reader for four-byte ISO/IEC 14443-A UIDs using MFRC522.
 * @details This teaching subset requests a card and reads cascade level 1. It does not
 * authenticate, read/write sectors, support seven-byte UIDs, or provide an access-control security
 * mechanism.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_MFRC522_H
#define AVR_DRIVER_MFRC522_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief UID length accepted by this driver subset. */
#define AVR_MFRC522_UID_LENGTH UINT8_C(4)

    /** @brief Caller-owned MFRC522 module pin configuration. */
    typedef struct
    {
        uint8_t chip_select_pin; /**< Uno GPIO connected to active-low SDA/SS/NSS. */
        uint8_t reset_pin;       /**< Uno GPIO connected to NRSTPD. */
        bool initialized;        /**< True after the SPI reader and antenna are configured. */
    } avr_mfrc522_t;

    /**
     * @brief Reset and configure the MFRC522 SPI interface and antenna driver.
     * @param[out] reader Caller-owned state; written only after successful setup.
     * @param[in] chip_select_pin Uno GPIO connected to module SDA/SS/NSS; cannot be D10..D13.
     * @param[in] reset_pin Uno GPIO connected to module RST; distinct from CS and D10..D13.
     * @return AVR_OK after setup or an argument/GPIO/SPI error.
     * @details Uses SPI mode 0 at F_CPU/4. The exact breakout must support its chosen supply and
     * signal levels; the IC itself is a low-voltage device. Does not enable global interrupts.
     */
    avr_status_t avr_mfrc522_init(avr_mfrc522_t *reader, uint8_t chip_select_pin,
                                  uint8_t reset_pin);

    /**
     * @brief Request a nearby Type A card and read its four-byte cascade-level-one UID.
     * @param[in] reader Initialized caller-owned MFRC522 state.
     * @param[out] uid Destination for four UID bytes; unchanged on failure.
     * @param[in] maximum_polls Maximum MFRC522 IRQ-register polls per protocol exchange; not time.
     * @return AVR_OK for a valid four-byte UID, AVR_ERROR_TIMEOUT when no card responds,
     * AVR_ERROR_BUSY on a collision, AVR_ERROR_UNSUPPORTED for a cascade/long UID or partial
     * response, or another HAL status for SPI/GPIO errors.
     * @details Does not select/authenticate the card or access memory blocks. A UID is an
     * identifier, not proof of identity; do not use this lesson as a door lock, payment system or
     * credential check. Polling is blocking but bounded. The caller owns the output array.
     */
    avr_status_t avr_mfrc522_read_uid(const avr_mfrc522_t *reader,
                                      uint8_t uid[AVR_MFRC522_UID_LENGTH], uint16_t maximum_polls);
#ifdef __cplusplus
}
#endif
#endif
