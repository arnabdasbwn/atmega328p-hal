/**
 * @file nmea_rmc.h
 * @brief Bounded parser for NMEA RMC position sentences.
 * @details Accepts standard $GPRMC and $GNRMC sentences with a valid two-digit XOR checksum.
 * Coordinates are returned as unsigned microdegrees with separate hemisphere letters.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_DRIVER_NMEA_RMC_H
#define AVR_DRIVER_NMEA_RMC_H
#include "avr_hal/common.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /** @brief Maximum supported NMEA line length before the terminating LF. */
#define AVR_NMEA_RMC_MAX_LENGTH UINT8_C(82)
    /** @brief Parser outcomes for one received byte. */
    typedef enum
    {
        AVR_NMEA_PENDING = 0, /**< More bytes are needed before a sentence can be checked. */
        AVR_NMEA_RECORD,      /**< A complete, checksummed RMC record was stored. */
        AVR_NMEA_UNSUPPORTED, /**< A complete valid sentence was not RMC. */
        AVR_NMEA_INVALID      /**< A complete sentence was malformed or had a bad checksum. */
    } avr_nmea_result_t;

    /** @brief Caller-owned incremental parser state. */
    typedef struct
    {
        char line[AVR_NMEA_RMC_MAX_LENGTH + 1U]; /**< Bounded sentence bytes plus terminator. */
        uint8_t length; /**< Number of buffered bytes, not including the final NUL. */
        bool active;    /**< True while bytes after a start marker are being collected. */
        bool overflow;  /**< Discard the current sentence after the line limit is exceeded. */
    } avr_nmea_rmc_parser_t;

    /** @brief Parsed RMC status and geographic coordinates. */
    typedef struct
    {
        bool fix_valid;                  /**< True when RMC status is A; false when status is V. */
        uint32_t latitude_microdegrees;  /**< Latitude magnitude in millionths of a degree. */
        uint32_t longitude_microdegrees; /**< Longitude magnitude in millionths of a degree. */
        char latitude_hemisphere;        /**< 'N' or 'S'; meaningful when fix_valid is true. */
        char longitude_hemisphere;       /**< 'E' or 'W'; meaningful when fix_valid is true. */
    } avr_nmea_rmc_record_t;

    /**
     * @brief Reset parser state before consuming a new stream.
     * @param[out] parser Non-null parser state owned by the caller.
     * @details Does not access a UART, allocate memory or retain external pointers.
     */
    void avr_nmea_rmc_reset(avr_nmea_rmc_parser_t *parser);

    /**
     * @brief Feed one byte and parse an RMC record at the line terminator.
     * @param[in,out] parser Active parser state; caller owns it and must serialize access.
     * @param[in] byte One received stream byte, including CR/LF when present.
     * @param[out] record Destination updated only for a complete valid RMC sentence; may be NULL
     * when the result is not AVR_NMEA_RECORD.
     * @return Pending, record, unsupported sentence or malformed/checksum error.
     * @details Constant bounded memory and execution per input byte; no heap or recursion. A line
     * longer than 82 bytes is discarded up to LF or the next '$' marker. Supports
     * active (A) and void (V) status. Coordinate fields are required for A status; V records may
     * omit them. Call reset before first use. Source stream and output storage remain caller-owned.
     */
    avr_nmea_result_t avr_nmea_rmc_feed(avr_nmea_rmc_parser_t *parser, uint8_t byte,
                                        avr_nmea_rmc_record_t *record);
#ifdef __cplusplus
}
#endif
#endif
