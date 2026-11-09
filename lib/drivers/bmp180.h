/**
 * @file bmp180.h
 * @brief BMP180 temperature and pressure measurement driver.
 * @details Uses the AVR HAL TWI transport and Bosch Sensortec BMP180 compensation equations.
 * The caller owns the bus between operations and selects one of four pressure oversampling modes.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_BMP180_H
#define AVR_DRIVER_BMP180_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Factory calibration coefficients read from sensor EEPROM. */
    typedef struct
    {
        int16_t ac1;  /**< Signed pressure coefficient AC1. */
        int16_t ac2;  /**< Signed pressure coefficient AC2. */
        int16_t ac3;  /**< Signed pressure coefficient AC3. */
        uint16_t ac4; /**< Unsigned pressure coefficient AC4. */
        uint16_t ac5; /**< Unsigned temperature coefficient AC5. */
        uint16_t ac6; /**< Unsigned temperature coefficient AC6. */
        int16_t b1;   /**< Signed pressure coefficient B1. */
        int16_t b2;   /**< Signed pressure coefficient B2. */
        int16_t mb;   /**< Signed pressure coefficient MB, retained for the data sheet record. */
        int16_t mc;   /**< Signed temperature coefficient MC. */
        int16_t md;   /**< Signed temperature coefficient MD. */
    } avr_bmp180_calibration_t;

    /** @brief Compensated BMP180 measurement in deci-degrees Celsius and pascals. */
    typedef struct
    {
        int16_t temperature_deci_c; /**< Temperature in 0.1 degrees Celsius. */
        uint32_t pressure_pa;       /**< Absolute pressure in pascals. */
    } avr_bmp180_measurement_t;

    /**
     * @brief Read and validate the factory calibration block.
     * @param[out] calibration Caller-owned output; unchanged if validation fails.
     * @return TWI status or AVR_ERROR_ARGUMENT for a null output or erased/zero calibration.
     * @details Requires initialized TWI at no more than 400 kHz. Reads address 0x77 and registers
     * 0xAA..0xBF. Foreground only; the bus must not be used concurrently.
     */
    avr_status_t avr_bmp180_init(avr_bmp180_calibration_t *calibration);

    /**
     * @brief Acquire and compensate one BMP180 measurement.
     * @param[in] calibration Valid coefficients from avr_bmp180_init; caller retains ownership.
     * @param[in] oversampling Pressure mode 0..3, from lowest power to highest resolution.
     * @param[out] measurement Caller-owned output; unchanged on transport or math failure.
     * @return TWI status, AVR_ERROR_ARGUMENT for invalid pointers/mode, or AVR_ERROR_IO for invalid
     * calibration/math results.
     * @details Performs blocking conversions with data-sheet maximum waits (5, 8, 14 or 26 ms) and
     * bounded TWI transactions. Foreground only; no global interrupts are changed. Temperature is
     * deci-degrees Celsius; pressure is absolute pascals.
     */
    avr_status_t avr_bmp180_read(const avr_bmp180_calibration_t *calibration, uint8_t oversampling,
                                 avr_bmp180_measurement_t *measurement);
#ifdef __cplusplus
}
#endif
#endif
