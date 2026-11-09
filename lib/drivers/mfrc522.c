/**
 * @file mfrc522.c
 * @brief ISO/IEC 14443-A request and four-byte UID anticollision over MFRC522 SPI.
 * @details Protocol polling uses caller-supplied finite bounds, receive storage is fixed-size, and
 * the implementation performs no card authentication or memory access.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "drivers/mfrc522.h"
#include "avr_hal/gpio.h"
#include "avr_hal/spi.h"
#include <util/delay.h>

/** @brief Maximum unsuccessful SPI completion polls for one byte. */
#define AVR_MFRC522_SPI_BUDGET UINT32_C(100000)
/** @brief Register address: command/status operation control. */
#define AVR_MFRC522_REG_COMMAND UINT8_C(0x01)
/** @brief Register address: interrupt request flags. */
#define AVR_MFRC522_REG_COM_IRQ UINT8_C(0x04)
/** @brief Register address: communication error flags. */
#define AVR_MFRC522_REG_ERROR UINT8_C(0x06)
/** @brief Register address: FIFO data port. */
#define AVR_MFRC522_REG_FIFO_DATA UINT8_C(0x09)
/** @brief Register address: FIFO occupancy and flush control. */
#define AVR_MFRC522_REG_FIFO_LEVEL UINT8_C(0x0A)
/** @brief Register address: final received-bit count. */
#define AVR_MFRC522_REG_CONTROL UINT8_C(0x0C)
/** @brief Register address: transceive framing and start bit. */
#define AVR_MFRC522_REG_BIT_FRAMING UINT8_C(0x0D)
/** @brief Register address: timer mode. */
#define AVR_MFRC522_REG_T_MODE UINT8_C(0x2A)
/** @brief Register address: timer prescaler. */
#define AVR_MFRC522_REG_T_PRESCALER UINT8_C(0x2B)
/** @brief Register address: timer reload high byte. */
#define AVR_MFRC522_REG_T_RELOAD_H UINT8_C(0x2C)
/** @brief Register address: timer reload low byte. */
#define AVR_MFRC522_REG_T_RELOAD_L UINT8_C(0x2D)
/** @brief Register address: transmitter modulation control. */
#define AVR_MFRC522_REG_TX_ASK UINT8_C(0x15)
/** @brief Register address: antenna transmitter control. */
#define AVR_MFRC522_REG_TX_CONTROL UINT8_C(0x14)
/** @brief Register address: CRC and communication mode. */
#define AVR_MFRC522_REG_MODE UINT8_C(0x11)
/** @brief MFRC522 command: software reset. */
#define AVR_MFRC522_CMD_SOFT_RESET UINT8_C(0x0F)
/** @brief MFRC522 command: transceive FIFO data with a Type A card. */
#define AVR_MFRC522_CMD_TRANSCEIVE UINT8_C(0x0C)
/** @brief MFRC522 IRQ bit set when a response has been received. */
#define AVR_MFRC522_IRQ_RX UINT8_C(0x20)
/** @brief MFRC522 IRQ bit set when the command engine becomes idle. */
#define AVR_MFRC522_IRQ_IDLE UINT8_C(0x10)
/** @brief MFRC522 IRQ bit set when the internal timer expires. */
#define AVR_MFRC522_IRQ_TIMER UINT8_C(0x01)
/** @brief MFRC522 ErrorReg bit indicating a receive collision. */
#define AVR_MFRC522_ERROR_COLLISION UINT8_C(0x08)
/** @brief MFRC522 ErrorReg bits indicating buffer or protocol faults. */
#define AVR_MFRC522_ERROR_FATAL UINT8_C(0x11)

/**
 * @brief Transfer one byte through the AVR SPI controller.
 * @param[in] output Byte sent MSB first.
 * @param[out] input Non-null byte destination.
 * @return HAL SPI status.
 */
static avr_status_t avr_mfrc522_exchange(uint8_t output, uint8_t *input)
{
    return avr_spi_transfer(output, input, AVR_MFRC522_SPI_BUDGET);
}

/**
 * @brief Write one MFRC522 register.
 * @param[in] reader Pin configuration; initialized state is not required during setup.
 * @param[in] address Register address in the range 0x00..0x3F.
 * @param[in] value Register value.
 * @return AVR_OK or the first GPIO/SPI error.
 */
static avr_status_t avr_mfrc522_write_register(const avr_mfrc522_t *reader, uint8_t address,
                                               uint8_t value)
{
    uint8_t ignored = 0U;
    avr_status_t status = avr_gpio_write(reader->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status =
            avr_mfrc522_exchange((uint8_t)(((uint32_t)address << 1U) & UINT32_C(0x7E)), &ignored);
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_exchange(value, &ignored);
    }
    if (avr_gpio_write(reader->chip_select_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    return status;
}

/**
 * @brief Read one MFRC522 register.
 * @param[in] reader Pin configuration; initialized state is not required during setup.
 * @param[in] address Register address in the range 0x00..0x3F.
 * @param[out] value Destination for the register value.
 * @return AVR_OK or the first GPIO/SPI error.
 */
static avr_status_t avr_mfrc522_read_register(const avr_mfrc522_t *reader, uint8_t address,
                                              uint8_t *value)
{
    uint8_t ignored = 0U;
    uint8_t result = 0U;
    avr_status_t status = avr_gpio_write(reader->chip_select_pin, false);
    if (status == AVR_OK)
    {
        status = avr_mfrc522_exchange(
            (uint8_t)((((uint32_t)address << 1U) & UINT32_C(0x7E)) | UINT32_C(0x80)), &ignored);
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_exchange(0U, &result);
    }
    if (avr_gpio_write(reader->chip_select_pin, true) != AVR_OK)
    {
        status = AVR_ERROR_IO;
    }
    if ((status == AVR_OK) && (value != NULL))
    {
        *value = result;
    }
    else if ((status == AVR_OK) && (value == NULL))
    {
        status = AVR_ERROR_ARGUMENT;
    }
    return status;
}

/**
 * @brief Validate reset/CS pin choices for the SPI reader.
 * @param[in] cs Chip-select pin.
 * @param[in] reset Hardware reset pin.
 * @return True when both Uno pins are valid, distinct and outside D10..D13.
 */
static bool avr_mfrc522_pins_valid(uint8_t cs, uint8_t reset)
{
    return ((cs < 20U) && (reset < 20U) && (cs != reset) && ((cs < 10U) || (cs > 13U)) &&
            ((reset < 10U) || (reset > 13U)));
}

/**
 * @brief Reset the chip and configure the SPI mode, timer and antenna.
 * @return AVR_OK after register configuration or an argument/GPIO/SPI error.
 * @details The timer supplies a finite no-card response interval; foreground IRQ polling adds a
 * second software bound. Configuration follows the MFRC522 register descriptions in the data sheet.
 * @see mfrc522.h for public pin and power requirements.
 */
avr_status_t avr_mfrc522_init(avr_mfrc522_t *reader, uint8_t chip_select_pin, uint8_t reset_pin)
{
    if ((reader == NULL) || !avr_mfrc522_pins_valid(chip_select_pin, reset_pin))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_mfrc522_t candidate = {chip_select_pin, reset_pin, false};
    avr_status_t status = avr_gpio_mode(chip_select_pin, AVR_GPIO_OUTPUT);
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
        status = avr_gpio_write(reset_pin, false);
    }
    if (status == AVR_OK)
    {
        status = avr_spi_init(0U, 4U);
    }
    if (status == AVR_OK)
    {
        _delay_ms(5);
        status = avr_gpio_write(reset_pin, true);
    }
    if (status == AVR_OK)
    {
        _delay_ms(50);
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_COMMAND,
                                            AVR_MFRC522_CMD_SOFT_RESET);
    }
    if (status == AVR_OK)
    {
        _delay_ms(50);
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_T_MODE, UINT8_C(0x8D));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_T_PRESCALER, UINT8_C(0x3E));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_T_RELOAD_L, UINT8_C(30));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_T_RELOAD_H, UINT8_C(0));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_TX_ASK, UINT8_C(0x40));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_MODE, UINT8_C(0x3D));
    }
    uint8_t tx_control = 0U;
    if (status == AVR_OK)
    {
        status = avr_mfrc522_read_register(&candidate, AVR_MFRC522_REG_TX_CONTROL, &tx_control);
    }
    if ((status == AVR_OK) && ((tx_control & UINT8_C(0x03)) != UINT8_C(0x03)))
    {
        status = avr_mfrc522_write_register(&candidate, AVR_MFRC522_REG_TX_CONTROL,
                                            (uint8_t)(tx_control | UINT8_C(0x03)));
    }
    if (status == AVR_OK)
    {
        candidate.initialized = true;
        *reader = candidate;
    }
    return status;
}

/**
 * @brief Exchange a bounded ISO14443-A command with the card and read at most five response bytes.
 * @param[in] reader Initialized MFRC522.
 * @param[in] transmit Transmit bytes held by caller for the duration of the call.
 * @param[in] transmit_length Number of bytes from one through two.
 * @param[in] last_byte_bits Number of valid bits in the final transmit byte, 0..7.
 * @param[out] receive Fixed five-byte temporary response storage.
 * @param[out] receive_length Number of complete received bytes.
 * @param[in] maximum_polls Finite IRQ-register polling bound.
 * @return AVR_OK, timeout, collision, overflow or another GPIO/SPI error.
 * @details The response is first read into a local array. No caller output is committed unless the
 * MFRC522 completes and reports a valid, byte-aligned response of at most five bytes.
 */
static avr_status_t avr_mfrc522_transceive(const avr_mfrc522_t *reader, const uint8_t *transmit,
                                           uint8_t transmit_length, uint8_t last_byte_bits,
                                           uint8_t receive[5], uint8_t *receive_length,
                                           uint16_t maximum_polls)
{
    if ((transmit == NULL) || (transmit_length == 0U) || (transmit_length > 2U) ||
        (last_byte_bits > 7U) || (receive == NULL) || (receive_length == NULL) ||
        (maximum_polls == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    avr_status_t status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_COMMAND, 0U);
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_COM_IRQ, UINT8_C(0x7F));
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_FIFO_LEVEL, UINT8_C(0x80));
    }
    for (uint8_t index = 0U; (index < transmit_length) && (status == AVR_OK); index++)
    {
        status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_FIFO_DATA, transmit[index]);
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_BIT_FRAMING, last_byte_bits);
    }
    if (status == AVR_OK)
    {
        status =
            avr_mfrc522_write_register(reader, AVR_MFRC522_REG_COMMAND, AVR_MFRC522_CMD_TRANSCEIVE);
    }
    if (status == AVR_OK)
    {
        status = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_BIT_FRAMING,
                                            (uint8_t)(last_byte_bits | UINT8_C(0x80)));
    }
    uint8_t irq = 0U;
    bool completed = false;
    for (uint16_t poll = 0U; (poll < maximum_polls) && (status == AVR_OK); poll++)
    {
        status = avr_mfrc522_read_register(reader, AVR_MFRC522_REG_COM_IRQ, &irq);
        if ((status == AVR_OK) &&
            ((irq & (AVR_MFRC522_IRQ_RX | AVR_MFRC522_IRQ_IDLE | AVR_MFRC522_IRQ_TIMER)) != 0U))
        {
            completed = true;
            break;
        }
    }
    if ((status == AVR_OK) && !completed)
    {
        status = AVR_ERROR_TIMEOUT;
    }
    if ((status == AVR_OK) && ((irq & AVR_MFRC522_IRQ_TIMER) != 0U))
    {
        status = AVR_ERROR_TIMEOUT;
    }
    uint8_t error = 0U;
    if (status == AVR_OK)
    {
        status = avr_mfrc522_read_register(reader, AVR_MFRC522_REG_ERROR, &error);
    }
    if ((status == AVR_OK) && ((error & AVR_MFRC522_ERROR_COLLISION) != 0U))
    {
        status = AVR_ERROR_BUSY;
    }
    if ((status == AVR_OK) && ((error & AVR_MFRC522_ERROR_FATAL) != 0U))
    {
        status = AVR_ERROR_IO;
    }
    uint8_t count = 0U;
    if (status == AVR_OK)
    {
        status = avr_mfrc522_read_register(reader, AVR_MFRC522_REG_FIFO_LEVEL, &count);
    }
    if ((status == AVR_OK) && (count > 5U))
    {
        status = AVR_ERROR_OVERFLOW;
    }
    uint8_t candidate[5] = {0U, 0U, 0U, 0U, 0U};
    for (uint8_t index = 0U; (index < count) && (status == AVR_OK); index++)
    {
        status = avr_mfrc522_read_register(reader, AVR_MFRC522_REG_FIFO_DATA, &candidate[index]);
    }
    uint8_t control = 0U;
    if (status == AVR_OK)
    {
        status = avr_mfrc522_read_register(reader, AVR_MFRC522_REG_CONTROL, &control);
    }
    if ((status == AVR_OK) && ((control & UINT8_C(0x07)) != 0U))
    {
        status = AVR_ERROR_UNSUPPORTED;
    }
    const avr_status_t cleanup = avr_mfrc522_write_register(reader, AVR_MFRC522_REG_COMMAND, 0U);
    if ((status == AVR_OK) && (cleanup != AVR_OK))
    {
        status = cleanup;
    }
    if (status == AVR_OK)
    {
        for (uint8_t index = 0U; index < count; index++)
        {
            receive[index] = candidate[index];
        }
        *receive_length = count;
    }
    return status;
}

/**
 * @brief Request one Type A card and check a four-byte UID cascade level.
 * @return AVR_OK when the anticollision response contains a valid four-byte UID and BCC.
 * @details This is an identifier read only. A collision means multiple cards need full
 * anticollision selection; a cascade tag indicates an unsupported longer UID form.
 * @see mfrc522.h for the public result and security limitations.
 */
avr_status_t avr_mfrc522_read_uid(const avr_mfrc522_t *reader, uint8_t uid[AVR_MFRC522_UID_LENGTH],
                                  uint16_t maximum_polls)
{
    if ((reader == NULL) || (uid == NULL) || !reader->initialized || (maximum_polls == 0U))
    {
        return AVR_ERROR_ARGUMENT;
    }
    const uint8_t request[] = {UINT8_C(0x52)};
    uint8_t response[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t response_length = 0U;
    avr_status_t status =
        avr_mfrc522_transceive(reader, request, 1U, 7U, response, &response_length, maximum_polls);
    if ((status == AVR_OK) && (response_length != 2U))
    {
        status = AVR_ERROR_UNSUPPORTED;
    }
    const uint8_t anticollision[] = {UINT8_C(0x93), UINT8_C(0x20)};
    if (status == AVR_OK)
    {
        status = avr_mfrc522_transceive(reader, anticollision, 2U, 0U, response, &response_length,
                                        maximum_polls);
    }
    if ((status == AVR_OK) && (response_length != 5U))
    {
        status = AVR_ERROR_UNSUPPORTED;
    }
    if ((status == AVR_OK) && (response[0] == UINT8_C(0x88)))
    {
        status = AVR_ERROR_UNSUPPORTED;
    }
    if (status == AVR_OK)
    {
        const uint8_t bcc = (uint8_t)(response[0] ^ response[1] ^ response[2] ^ response[3]);
        if (bcc != response[4])
        {
            status = AVR_ERROR_IO;
        }
    }
    if (status == AVR_OK)
    {
        for (uint8_t index = 0U; index < AVR_MFRC522_UID_LENGTH; index++)
        {
            uid[index] = response[index];
        }
    }
    return status;
}
