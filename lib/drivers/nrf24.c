/**
 * @file nrf24.c
 * @brief Single-pipe, fixed-payload nRF24L01+ SPI endpoint.
 * @details Configures the Nordic Enhanced ShockBurst path with finite retries and bounded
 * foreground polling. The driver uses one fixed address and does not allocate memory dynamically.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "drivers/nrf24.h"
#include "avr_hal/gpio.h"
#include "avr_hal/spi.h"
#include <util/delay.h>

/** @brief Maximum unsuccessful SPI polls for one register byte. */
#define AVR_NRF24_SPI_BUDGET UINT32_C(100000)
/** @brief Command: write one or more bytes to a configuration register. */
#define AVR_NRF24_CMD_W_REGISTER UINT8_C(0x20)
/** @brief Command: read the status register without changing device state. */
#define AVR_NRF24_CMD_NOP UINT8_C(0xFF)
/** @brief Command: write a fixed payload to the transmit FIFO. */
#define AVR_NRF24_CMD_W_TX_PAYLOAD UINT8_C(0xA0)
/** @brief Command: read one fixed payload from the receive FIFO. */
#define AVR_NRF24_CMD_R_RX_PAYLOAD UINT8_C(0x61)
/** @brief Command: clear the transmit FIFO. */
#define AVR_NRF24_CMD_FLUSH_TX UINT8_C(0xE1)
/** @brief Command: clear the receive FIFO. */
#define AVR_NRF24_CMD_FLUSH_RX UINT8_C(0xE2)
/** @brief Register address for configuration and operating mode. */
#define AVR_NRF24_REG_CONFIG UINT8_C(0x00)
/** @brief Register address for enabled auto acknowledgement pipes. */
#define AVR_NRF24_REG_EN_AA UINT8_C(0x01)
/** @brief Register address for enabled receive pipes. */
#define AVR_NRF24_REG_EN_RXADDR UINT8_C(0x02)
/** @brief Register address for five-byte address width. */
#define AVR_NRF24_REG_SETUP_AW UINT8_C(0x03)
/** @brief Register address for retransmit delay and count. */
#define AVR_NRF24_REG_SETUP_RETR UINT8_C(0x04)
/** @brief Register address for 2.4 GHz channel. */
#define AVR_NRF24_REG_RF_CH UINT8_C(0x05)
/** @brief Register address for RF data rate and power. */
#define AVR_NRF24_REG_RF_SETUP UINT8_C(0x06)
/** @brief Register address for interrupt/status flags. */
#define AVR_NRF24_REG_STATUS UINT8_C(0x07)
/** @brief Register address for pipe 0 receive address. */
#define AVR_NRF24_REG_RX_ADDR_P0 UINT8_C(0x0A)
/** @brief Register address for transmit address. */
#define AVR_NRF24_REG_TX_ADDR UINT8_C(0x10)
/** @brief Register address for pipe 0 fixed payload width. */
#define AVR_NRF24_REG_RX_PW_P0 UINT8_C(0x11)
/** @brief STATUS bit: receive FIFO has unread data. */
#define AVR_NRF24_STATUS_RX_DR UINT8_C(0x40)
/** @brief STATUS bit: transmit completed and automatic acknowledgement received. */
#define AVR_NRF24_STATUS_TX_DS UINT8_C(0x20)
/** @brief STATUS bit: configured retransmissions were exhausted. */
#define AVR_NRF24_STATUS_MAX_RT UINT8_C(0x10)

/**
 * @brief Exchange one byte using the AVR SPI peripheral.
 * @param[in] output Byte transmitted MSB first.
 * @param[out] input Non-null received-byte destination.
 * @return Status from the HAL SPI transfer.
 */
static avr_status_t avr_nrf24_exchange(uint8_t output, uint8_t *input)
{
    return avr_spi_transfer(output, input, AVR_NRF24_SPI_BUDGET);
}

/**
 * @brief Send an SPI command and its optional data under one CSN assertion.
 * @param[in] radio Initialized pin configuration.
 * @param[in] command Command byte.
 * @param[in] transmit Optional bytes to send; NULL is allowed when length is zero.
 * @param[out] receive Optional receive array; NULL discards incoming bytes.
 * @param[in] length Number of data bytes after the command.
 * @return First GPIO/SPI failure or AVR_OK.
 * @details CSN is raised on every return path. The command's returned STATUS byte is discarded.
 */
static avr_status_t avr_nrf24_transaction(const avr_nrf24_t *radio, uint8_t command,
                                          const uint8_t *transmit, uint8_t *receive, uint8_t length)
{
    uint8_t ignored = 0U;
    avr_status_t status = avr_gpio_write(radio->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status = avr_nrf24_exchange(command, &ignored);
    }
    for (uint8_t index = 0U; (index < length) && (status == AVR_OK); index++)
    {
        const uint8_t output = (transmit != NULL) ? transmit[index] : AVR_NRF24_CMD_NOP;
        uint8_t input = 0U;
        status = avr_nrf24_exchange(output, &input);
        if ((status == AVR_OK) && (receive != NULL))
        {
            receive[index] = input;
        }
    }
    if (avr_gpio_write(radio->chip_select_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    return status;
}

/**
 * @brief Write bytes to a radio register.
 * @param[in] radio Initialized radio pins.
 * @param[in] address Register address from 0x00 through 0x1F.
 * @param[in] data Caller-owned register byte sequence.
 * @param[in] length Number of bytes, 1 through five.
 * @return AVR_OK or the first SPI/GPIO error.
 */
static avr_status_t avr_nrf24_write_register(const avr_nrf24_t *radio, uint8_t address,
                                             const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0U) || (length > AVR_NRF24_ADDRESS_LENGTH))
    {
        return AVR_ERROR_ARGUMENT;
    }
    return avr_nrf24_transaction(radio, (uint8_t)(AVR_NRF24_CMD_W_REGISTER | (address & 0x1FU)),
                                 data, NULL, length);
}

/**
 * @brief Read the radio STATUS register by sending NOP.
 * @param[in] radio Initialized radio pins.
 * @param[out] status_out Destination for the returned STATUS byte.
 * @return AVR_OK or the first SPI/GPIO error.
 */
static avr_status_t avr_nrf24_read_status(const avr_nrf24_t *radio, uint8_t *status_out)
{
    uint8_t status = 0U;
    avr_status_t result = avr_gpio_write(radio->chip_select_pin, false);
    if (result == AVR_OK)
    {
        result = avr_nrf24_exchange(AVR_NRF24_CMD_NOP, &status);
    }
    if (avr_gpio_write(radio->chip_select_pin, true) != AVR_OK)
    {
        result = AVR_ERROR_IO;
    }
    if ((result == AVR_OK) && (status_out != NULL))
    {
        *status_out = status;
    }
    else if ((result == AVR_OK) && (status_out == NULL))
    {
        result = AVR_ERROR_ARGUMENT;
    }
    return result;
}

/**
 * @brief Issue a no-data command under one CSN assertion.
 * @param[in] radio Initialized radio pins.
 * @param[in] command Flush or other command byte.
 * @return AVR_OK or the first GPIO/SPI error.
 */
static avr_status_t avr_nrf24_command_only(const avr_nrf24_t *radio, uint8_t command)
{
    return avr_nrf24_transaction(radio, command, NULL, NULL, 0U);
}

/**
 * @brief Clear selected write-one-to-clear interrupt flags.
 * @param[in] radio Initialized radio pins.
 * @param[in] flags STATUS bits to clear.
 * @return AVR_OK or the first SPI/GPIO error.
 * @details STATUS flags are cleared by writing a one to each selected bit.
 */
static avr_status_t avr_nrf24_clear_status(const avr_nrf24_t *radio, uint8_t flags)
{
    return avr_nrf24_write_register(radio, AVR_NRF24_REG_STATUS, &flags, 1U);
}

/**
 * @brief Validate caller-selected CE and CSN pins.
 * @param[in] ce Chip-enable pin.
 * @param[in] csn Chip-select pin.
 * @return True when both pins are valid, distinct, and outside SPI-reserved D10..D13.
 */
static bool avr_nrf24_pins_valid(uint8_t ce, uint8_t csn)
{
    return ((ce < 20U) && (csn < 20U) && (ce != csn) && ((ce < 10U) || (ce > 13U)) &&
            ((csn < 10U) || (csn > 13U)));
}

/**
 * @brief Set up a fixed-width pipe-0 radio endpoint.
 * @return AVR_OK or an argument/GPIO/SPI error.
 * @details Sets channel 76, 1 Mbps, 0 dBm, CRC-16, five-byte addresses, fixed five-byte payloads,
 * auto-ack on pipe 0, and at most 15 retransmissions with 750 microseconds between attempts.
 * @see nrf24.h for complete parameter, ownership, and result contracts.
 */
avr_status_t avr_nrf24_init(avr_nrf24_t *radio, uint8_t ce_pin, uint8_t csn_pin,
                            const uint8_t address[AVR_NRF24_ADDRESS_LENGTH], bool receiver)
{
    if ((radio == NULL) || (address == NULL) || !avr_nrf24_pins_valid(ce_pin, csn_pin))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_nrf24_t candidate = {ce_pin, csn_pin, {0U, 0U, 0U, 0U, 0U}, receiver, false};
    for (uint8_t index = 0U; index < AVR_NRF24_ADDRESS_LENGTH; index++)
    {
        candidate.address[index] = address[index];
    }
    avr_status_t status = avr_gpio_mode(ce_pin, AVR_GPIO_OUTPUT);
    if (status == AVR_OK)
    {
        status = avr_gpio_mode(csn_pin, AVR_GPIO_OUTPUT);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(ce_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(csn_pin, true);
    }
    if (status == AVR_OK)
    {
        status = avr_spi_init(0U, 4U);
    }
    if (status != AVR_OK)
    {
        return status;
    }

    const uint8_t powered_down = UINT8_C(0x0C);
    const uint8_t auto_ack = UINT8_C(0x01);
    const uint8_t receive_pipes = UINT8_C(0x01);
    const uint8_t address_width = UINT8_C(0x03);
    const uint8_t retry_setup = UINT8_C(0x2F);
    const uint8_t channel = UINT8_C(76);
    const uint8_t rf_setup = UINT8_C(0x06);
    const uint8_t payload_width = AVR_NRF24_PAYLOAD_LENGTH;
    const uint8_t clear_flags = UINT8_C(0x70);
    status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_CONFIG, &powered_down, 1U);
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_EN_AA, &auto_ack, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_EN_RXADDR, &receive_pipes, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_SETUP_AW, &address_width, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_SETUP_RETR, &retry_setup, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_RF_CH, &channel, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_RF_SETUP, &rf_setup, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_RX_ADDR_P0, candidate.address,
                                          AVR_NRF24_ADDRESS_LENGTH);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_TX_ADDR, candidate.address,
                                          AVR_NRF24_ADDRESS_LENGTH);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_RX_PW_P0, &payload_width, 1U);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_clear_status(&candidate, clear_flags);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_command_only(&candidate, AVR_NRF24_CMD_FLUSH_TX);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_command_only(&candidate, AVR_NRF24_CMD_FLUSH_RX);
    }
    const uint8_t active_config = (uint8_t)(UINT8_C(0x0E) | (receiver ? UINT8_C(0x01) : 0U));
    if (status == AVR_OK)
    {
        status = avr_nrf24_write_register(&candidate, AVR_NRF24_REG_CONFIG, &active_config, 1U);
    }
    if (status == AVR_OK)
    {
        _delay_ms(2);
        candidate.initialized = true;
        if (receiver)
        {
            status = avr_gpio_write(ce_pin, true);
        }
        if (status == AVR_OK)
        {
            *radio = candidate;
        }
    }
    return status;
}

/**
 * @brief Send one fixed-size payload and poll for TX_DS or MAX_RT.
 * @return AVR_OK on acknowledged send, AVR_ERROR_IO on exhausted retries, or another HAL status.
 * @details The CE pulse starts the transmission. Polling stops at the caller's finite bound.
 * @see nrf24.h for public argument and output contracts.
 */
avr_status_t avr_nrf24_transmit(const avr_nrf24_t *radio,
                                const uint8_t payload[AVR_NRF24_PAYLOAD_LENGTH],
                                uint16_t maximum_polls)
{
    if ((radio == NULL) || (payload == NULL) || !radio->initialized || radio->receiver ||
        (maximum_polls == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_status_t status = avr_gpio_write(radio->chip_enable_pin, false);
    if (status == AVR_OK)
    {
        status = avr_nrf24_command_only(radio, AVR_NRF24_CMD_FLUSH_TX);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_clear_status(
            radio, (uint8_t)(AVR_NRF24_STATUS_TX_DS | AVR_NRF24_STATUS_MAX_RT));
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_transaction(radio, AVR_NRF24_CMD_W_TX_PAYLOAD, payload, NULL,
                                       AVR_NRF24_PAYLOAD_LENGTH);
    }
    if (status == AVR_OK)
    {
        status = avr_gpio_write(radio->chip_enable_pin, true);
    }
    if (status == AVR_OK)
    {
        _delay_us(15);
        status = avr_gpio_write(radio->chip_enable_pin, false);
    }
    for (uint16_t poll = 0U; (poll < maximum_polls) && (status == AVR_OK); poll++)
    {
        uint8_t radio_status = 0U;
        status = avr_nrf24_read_status(radio, &radio_status);
        if ((status == AVR_OK) && ((radio_status & AVR_NRF24_STATUS_TX_DS) != 0U))
        {
            status = avr_nrf24_clear_status(radio, AVR_NRF24_STATUS_TX_DS);
            return status;
        }
        if ((status == AVR_OK) && ((radio_status & AVR_NRF24_STATUS_MAX_RT) != 0U))
        {
            (void)avr_nrf24_clear_status(radio, AVR_NRF24_STATUS_MAX_RT);
            (void)avr_nrf24_command_only(radio, AVR_NRF24_CMD_FLUSH_TX);
            return AVR_ERROR_IO;
        }
    }
    if (status == AVR_OK)
    {
        status = AVR_ERROR_TIMEOUT;
    }
    (void)avr_gpio_write(radio->chip_enable_pin, false);
    return status;
}

/**
 * @brief Receive one fixed-width payload with a finite status-poll bound.
 * @return AVR_OK after copying one payload, AVR_ERROR_TIMEOUT when the bound expires, or HAL error.
 * @details The caller's buffer is unchanged unless the entire payload is read successfully.
 * @see nrf24.h for public argument and output contracts.
 */
avr_status_t avr_nrf24_receive(const avr_nrf24_t *radio, uint8_t payload[AVR_NRF24_PAYLOAD_LENGTH],
                               uint16_t maximum_polls)
{
    if ((radio == NULL) || (payload == NULL) || !radio->initialized || !radio->receiver ||
        (maximum_polls == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_status_t status = AVR_OK;
    bool data_ready = false;
    for (uint16_t poll = 0U; poll < maximum_polls; poll++)
    {
        uint8_t radio_status = 0U;
        status = avr_nrf24_read_status(radio, &radio_status);
        if (status != AVR_OK)
        {
            return status;
        }
        if ((radio_status & AVR_NRF24_STATUS_RX_DR) != 0U)
        {
            data_ready = true;
            break;
        }
    }
    if (status != AVR_OK)
    {
        return status;
    }
    if (!data_ready)
    {
        return AVR_ERROR_TIMEOUT;
    }
    uint8_t candidate[AVR_NRF24_PAYLOAD_LENGTH];
    status = avr_gpio_write(radio->chip_enable_pin, false);
    if (status == AVR_OK)
    {
        status = avr_nrf24_transaction(radio, AVR_NRF24_CMD_R_RX_PAYLOAD, NULL, candidate,
                                       AVR_NRF24_PAYLOAD_LENGTH);
    }
    if (status == AVR_OK)
    {
        status = avr_nrf24_clear_status(radio, AVR_NRF24_STATUS_RX_DR);
    }
    if (avr_gpio_write(radio->chip_enable_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    if (status == AVR_OK)
    {
        for (uint8_t index = 0U; index < AVR_NRF24_PAYLOAD_LENGTH; index++)
        {
            payload[index] = candidate[index];
        }
    }
    return status;
}
