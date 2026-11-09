/**
 * @file bmp180.c
 * @brief BMP180 calibration, conversion and fixed-point compensation implementation.
 * @details Implements the measurement sequence and integer equations in Bosch Sensortec document
 * BST-BMP180-DS000-12, revision 2.8. I2C traffic uses the bounded shared AVR HAL.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "bmp180.h"
#include "avr_hal/twi.h"
#include "register_device.h"
#include <util/delay.h>

/** @brief Default seven-bit I2C address selected for the BMP180. */
#define AVR_BMP180_ADDRESS UINT8_C(0x77)
/** @brief First register of the 22-byte factory calibration block. */
#define AVR_BMP180_CALIBRATION_REGISTER UINT8_C(0xAA)
/** @brief Control register used to start temperature and pressure conversions. */
#define AVR_BMP180_CONTROL_REGISTER UINT8_C(0xF4)
/** @brief First register of the uncompensated conversion data. */
#define AVR_BMP180_DATA_REGISTER UINT8_C(0xF6)

/**
 * @brief Convert a signed big-endian coefficient pair without an out-of-range unsigned cast.
 * @param[in] bytes Pointer to two coefficient bytes in sensor order.
 * @return Sign-extended coefficient in the range -32768..32767.
 */
static int16_t avr_bmp180_signed_word(const uint8_t *bytes)
{
    uint16_t raw = (uint16_t)(((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1]);
    int32_t value = (raw <= INT16_MAX) ? (int32_t)raw : ((int32_t)raw - INT32_C(65536));
    return (int16_t)value;
}

/**
 * @brief Convert an unsigned big-endian coefficient pair.
 * @param[in] bytes Pointer to two coefficient bytes in sensor order.
 * @return Coefficient in the range 0..65535.
 */
static uint16_t avr_bmp180_unsigned_word(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1]);
}

/**
 * @brief Perform a portable floor-style arithmetic right shift on a signed value.
 * @param[in] value Signed 64-bit value to scale.
 * @param[in] count Shift-equivalent divisor exponent, from 0 to 62.
 * @return Floor(value / 2^count), matching arithmetic right-shift rounding.
 */
static int64_t avr_bmp180_shift_right(int64_t value, uint8_t count)
{
    int64_t divisor = INT64_C(1) << count;
    int64_t quotient = value / divisor;
    int64_t remainder = value % divisor;
    if ((value < 0) && (remainder != 0))
    {
        quotient -= INT64_C(1);
    }
    return quotient;
}

/**
 * @brief Read a single uncompensated temperature sample after a bounded conversion delay.
 * @param[out] raw Non-null destination, updated only after a successful register read.
 * @return Status from the bounded TWI write/read sequence.
 */
static avr_status_t avr_bmp180_read_raw_temperature(uint16_t *raw)
{
    uint8_t bytes[2];
    avr_status_t status =
        avr_device_write(AVR_BMP180_ADDRESS, AVR_BMP180_CONTROL_REGISTER, UINT8_C(0x2E));
    if (status != AVR_OK)
    {
        return status;
    }
    _delay_ms(5);
    status = avr_device_read(AVR_BMP180_ADDRESS, AVR_BMP180_DATA_REGISTER, bytes, 2U);
    if (status == AVR_OK)
    {
        *raw = avr_bmp180_unsigned_word(bytes);
    }
    return status;
}

/**
 * @brief Read one pressure sample using the selected oversampling conversion.
 * @param[in] oversampling Valid sensor setting in the range 0..3.
 * @param[out] raw Non-null destination, updated only after a successful register read.
 * @return Status from the bounded TWI write/read sequence.
 */
static avr_status_t avr_bmp180_read_raw_pressure(uint8_t oversampling, uint32_t *raw)
{
    uint8_t command = (uint8_t)(UINT8_C(0x34) | (uint8_t)(oversampling << 6U));
    uint8_t bytes[3];
    avr_status_t status =
        avr_device_write(AVR_BMP180_ADDRESS, AVR_BMP180_CONTROL_REGISTER, command);
    if (status != AVR_OK)
    {
        return status;
    }
    switch (oversampling)
    {
    case 0U:
        _delay_ms(5);
        break;
    case 1U:
        _delay_ms(8);
        break;
    case 2U:
        _delay_ms(14);
        break;
    default:
        _delay_ms(26);
        break;
    }
    status = avr_device_read(AVR_BMP180_ADDRESS, AVR_BMP180_DATA_REGISTER, bytes, 3U);
    if (status == AVR_OK)
    {
        uint32_t packed = ((uint32_t)bytes[0] << 16U) | ((uint32_t)bytes[1] << 8U) | bytes[2];
        *raw = packed >> (uint8_t)(8U - oversampling);
    }
    return status;
}

/**
 * @brief Check that coefficients required as divisors are not zero.
 * @param[in] calibration Candidate coefficient set.
 * @return True when all coefficients used as divisors are nonzero.
 */
static bool avr_bmp180_calibration_usable(const avr_bmp180_calibration_t *calibration)
{
    return (calibration->ac4 != 0U) && (calibration->ac5 != 0U) && (calibration->ac6 != 0U) &&
           (calibration->md != 0);
}

avr_status_t avr_bmp180_init(avr_bmp180_calibration_t *calibration)
{
    uint8_t bytes[22];
    avr_bmp180_calibration_t candidate;
    avr_status_t status;
    if (calibration == NULL)
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_device_read(AVR_BMP180_ADDRESS, AVR_BMP180_CALIBRATION_REGISTER, bytes,
                             (uint8_t)sizeof(bytes));
    if (status != AVR_OK)
    {
        return status;
    }
    candidate.ac1 = avr_bmp180_signed_word(&bytes[0]);
    candidate.ac2 = avr_bmp180_signed_word(&bytes[2]);
    candidate.ac3 = avr_bmp180_signed_word(&bytes[4]);
    candidate.ac4 = avr_bmp180_unsigned_word(&bytes[6]);
    candidate.ac5 = avr_bmp180_unsigned_word(&bytes[8]);
    candidate.ac6 = avr_bmp180_unsigned_word(&bytes[10]);
    candidate.b1 = avr_bmp180_signed_word(&bytes[12]);
    candidate.b2 = avr_bmp180_signed_word(&bytes[14]);
    candidate.mb = avr_bmp180_signed_word(&bytes[16]);
    candidate.mc = avr_bmp180_signed_word(&bytes[18]);
    candidate.md = avr_bmp180_signed_word(&bytes[20]);
    if (!avr_bmp180_calibration_usable(&candidate))
    {
        return AVR_ERROR_IO;
    }
    *calibration = candidate;
    return AVR_OK;
}

avr_status_t avr_bmp180_read(const avr_bmp180_calibration_t *calibration, uint8_t oversampling,
                             avr_bmp180_measurement_t *measurement)
{
    uint16_t uncompensated_temperature = 0U;
    uint32_t uncompensated_pressure = 0U;
    int64_t x1, x2, b5, b6, x3, b3, b4, b7, pressure;
    avr_status_t status;
    avr_bmp180_measurement_t candidate;
    if ((calibration == NULL) || (measurement == NULL) || (oversampling > 3U) ||
        !avr_bmp180_calibration_usable(calibration))
    {
        return AVR_ERROR_ARGUMENT;
    }
    status = avr_bmp180_read_raw_temperature(&uncompensated_temperature);
    if (status != AVR_OK)
    {
        return status;
    }
    status = avr_bmp180_read_raw_pressure(oversampling, &uncompensated_pressure);
    if (status != AVR_OK)
    {
        return status;
    }

    x1 = avr_bmp180_shift_right(
        ((int64_t)uncompensated_temperature - calibration->ac6) * calibration->ac5, 15U);
    if ((x1 + calibration->md) == 0)
    {
        return AVR_ERROR_IO;
    }
    x2 = ((int64_t)calibration->mc * INT64_C(2048)) / (x1 + calibration->md);
    b5 = x1 + x2;
    x1 = avr_bmp180_shift_right(b5 + 8, 4U);
    if ((x1 < INT16_MIN) || (x1 > INT16_MAX))
    {
        return AVR_ERROR_IO;
    }
    candidate.temperature_deci_c = (int16_t)x1;

    b6 = b5 - INT64_C(4000);
    x1 = avr_bmp180_shift_right((int64_t)calibration->b2 * avr_bmp180_shift_right(b6 * b6, 12U),
                                11U);
    x2 = avr_bmp180_shift_right((int64_t)calibration->ac2 * b6, 11U);
    x3 = x1 + x2;
    b3 = avr_bmp180_shift_right(
        (((int64_t)calibration->ac1 * 4 + x3) * (INT64_C(1) << oversampling)) + 2, 2U);
    x1 = avr_bmp180_shift_right((int64_t)calibration->ac3 * b6, 13U);
    x2 = avr_bmp180_shift_right((int64_t)calibration->b1 * avr_bmp180_shift_right(b6 * b6, 12U),
                                16U);
    x3 = avr_bmp180_shift_right(x1 + x2 + 2, 2U);
    b4 = avr_bmp180_shift_right((int64_t)calibration->ac4 * (x3 + INT64_C(32768)), 15U);
    b7 = ((int64_t)uncompensated_pressure - b3) * (INT64_C(50000) >> oversampling);
    if (b4 <= 0)
    {
        return AVR_ERROR_IO;
    }
    if (b7 < INT64_C(0x80000000))
    {
        pressure = (b7 * 2) / b4;
    }
    else
    {
        pressure = (b7 / b4) * 2;
    }
    x1 = avr_bmp180_shift_right(pressure, 8U);
    x1 = avr_bmp180_shift_right(x1 * x1 * 3038, 16U);
    x2 = avr_bmp180_shift_right(-7357 * pressure, 16U);
    pressure += avr_bmp180_shift_right(x1 + x2 + 3791, 4U);
    if ((pressure <= 0) || (pressure > UINT32_MAX))
    {
        return AVR_ERROR_IO;
    }
    candidate.pressure_pa = (uint32_t)pressure;
    *measurement = candidate;
    return AVR_OK;
}
