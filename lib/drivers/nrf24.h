/**
 * @file nrf24.h
 * @brief Fixed-width, bounded polling API for one nRF24L01+ data pipe.
 * @details The driver supports one five-byte address and fixed five-byte payloads. It uses the AVR
 * HAL SPI controller, owns caller-selected CE/CSN pins, and keeps no dynamic memory.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_NRF24_H
#define AVR_DRIVER_NRF24_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Address width used by the teaching radio pair. */
#define AVR_NRF24_ADDRESS_LENGTH UINT8_C(5)
    /** @brief Fixed payload width used by the teaching radio pair. */
#define AVR_NRF24_PAYLOAD_LENGTH UINT8_C(5)

    /** @brief Caller-owned radio pin, address and role configuration. */
    typedef struct
    {
        uint8_t chip_enable_pin;                   /**< Uno GPIO connected to active-high CE. */
        uint8_t chip_select_pin;                   /**< Uno GPIO connected to active-low CSN. */
        uint8_t address[AVR_NRF24_ADDRESS_LENGTH]; /**< Shared pipe-0 address, copied at setup. */
        bool receiver;    /**< True selects PRX role; false selects PTX role. */
        bool initialized; /**< True after register configuration completes. */
    } avr_nrf24_t;

    /**
     * @brief Configure a 1 Mbps, channel-76, five-byte-address radio endpoint.
     * @param[out] radio Caller-owned state, written only after successful initialization.
     * @param[in] ce_pin Uno GPIO connected to CE; distinct from CSN and D10..D13.
     * @param[in] csn_pin Uno GPIO connected to active-low CSN; distinct from CE and D10..D13.
     * @param[in] address Exactly five address bytes shared by both endpoints.
     * @param[in] receiver True configures receive mode; false configures transmit mode.
     * @return AVR_OK after configuration or an argument/GPIO/SPI error.
     * @details Enables CRC-16, auto-ack on pipe 0, a bounded retransmission policy, 1 Mbps air
     * rate, and a fixed five-byte payload. Use a known genuine-compatible module and an appropriate
     * 3.3 V supply. It does not enable global interrupts.
     */
    avr_status_t avr_nrf24_init(avr_nrf24_t *radio, uint8_t ce_pin, uint8_t csn_pin,
                                const uint8_t address[AVR_NRF24_ADDRESS_LENGTH], bool receiver);

    /**
     * @brief Transmit one fixed-width payload and wait a bounded number of status polls.
     * @param[in] radio Initialized PTX endpoint.
     * @param[in] payload Caller-owned array of exactly five bytes, copied into the TX FIFO.
     * @param[in] maximum_polls Maximum status queries before returning timeout; not milliseconds.
     * @return AVR_OK on TX_DS, AVR_ERROR_IO after MAX_RT, timeout, or an argument/SPI error.
     * @details Blocking foreground operation. Auto-retries are bounded by the configured radio
     * retry register. No payload pointer is retained after return.
     */
    avr_status_t avr_nrf24_transmit(const avr_nrf24_t *radio,
                                    const uint8_t payload[AVR_NRF24_PAYLOAD_LENGTH],
                                    uint16_t maximum_polls);

    /**
     * @brief Wait for and copy one fixed-width payload from the RX FIFO.
     * @param[in] radio Initialized PRX endpoint.
     * @param[out] payload Destination for exactly five received bytes; unchanged on failure.
     * @param[in] maximum_polls Maximum status queries before returning timeout; not milliseconds.
     * @return AVR_OK on a received payload, timeout, or an argument/SPI error.
     * @details Blocking foreground operation. The driver copies through a local five-byte buffer
     * and clears RX_DR before restoring CE high. This API is not reentrant and does not use IRQ.
     */
    avr_status_t avr_nrf24_receive(const avr_nrf24_t *radio,
                                   uint8_t payload[AVR_NRF24_PAYLOAD_LENGTH],
                                   uint16_t maximum_polls);
#ifdef __cplusplus
}
#endif
#endif
