/**
 * @file nmea_rmc.c
 * @brief Bounded NMEA RMC checksum and coordinate parser.
 * @details Parses field spans directly from a fixed line buffer. It accepts a record only after
 * the XOR checksum and required status/coordinate fields have passed validation.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#include "nmea_rmc.h"
#include <string.h>

/** @brief Maximum comma-separated fields retained from one checked sentence. */
#define AVR_NMEA_MAX_FIELDS UINT8_C(12)

/**
 * @brief Check whether a byte is an ASCII decimal digit.
 * @param[in] value Character to inspect.
 * @return True for the ASCII range '0' through '9'.
 */
static bool avr_nmea_is_digit(char value)
{
    return (value >= '0') && (value <= '9');
}

/**
 * @brief Convert one ASCII hexadecimal digit to its numeric value.
 * @param[in] value Candidate input character.
 * @param[out] result Destination updated only when the character is hexadecimal.
 * @return True for 0..9, A..F or a..f; otherwise false.
 */
static bool avr_nmea_hex_value(char value, uint8_t *result)
{
    uint8_t converted;
    if ((result == NULL))
    {
        return false;
    }
    if ((value >= '0') && (value <= '9'))
    {
        converted = (uint8_t)(value - '0');
    }
    else if ((value >= 'A') && (value <= 'F'))
    {
        converted = (uint8_t)(value - 'A' + 10);
    }
    else if ((value >= 'a') && (value <= 'f'))
    {
        converted = (uint8_t)(value - 'a' + 10);
    }
    else
    {
        return false;
    }
    *result = converted;
    return true;
}

/**
 * @brief Convert a bounded NMEA degrees/minutes coordinate to microdegrees.
 * @param[in] line NUL-terminated parser buffer.
 * @param[in] start Inclusive first field index.
 * @param[in] end Exclusive field end index.
 * @param[in] degree_digits Two for latitude or three for longitude.
 * @param[in] maximum_degrees Maximum legal magnitude, 90 or 180.
 * @param[out] coordinate Caller-owned microdegree result.
 * @return True only for a complete, in-range coordinate with at most six decimal minute digits.
 */
static bool avr_nmea_coordinate(const char *line, uint8_t start, uint8_t end, uint8_t degree_digits,
                                uint16_t maximum_degrees, uint32_t *coordinate)
{
    uint8_t index = start;
    uint8_t decimal_count = 0U;
    uint32_t degrees = 0U;
    uint32_t minutes = 0U;
    uint32_t fraction = 0U;
    uint32_t minute_scale = UINT32_C(1);
    bool decimal_seen = false;
    uint64_t minute_microdegrees;
    uint64_t result;
    if ((line == NULL) || (coordinate == NULL) || (degree_digits < 2U) || (degree_digits > 3U) ||
        ((uint16_t)(end - start) < (uint16_t)(degree_digits + 2U)))
    {
        return false;
    }
    for (uint8_t digit = 0U; digit < degree_digits; digit++)
    {
        if (!avr_nmea_is_digit(line[index]))
        {
            return false;
        }
        degrees = (degrees * UINT32_C(10)) + (uint8_t)(line[index] - '0');
        index++;
    }
    for (uint8_t digit = 0U; digit < 2U; digit++)
    {
        if (!avr_nmea_is_digit(line[index]))
        {
            return false;
        }
        minutes = (minutes * UINT32_C(10)) + (uint8_t)(line[index] - '0');
        index++;
    }
    if ((index < end) && (line[index] == '.'))
    {
        decimal_seen = true;
        index++;
        while (index < end)
        {
            if (!avr_nmea_is_digit(line[index]) || (decimal_count >= 6U))
            {
                return false;
            }
            fraction = (fraction * UINT32_C(10)) + (uint8_t)(line[index] - '0');
            decimal_count++;
            minute_scale *= UINT32_C(10);
            index++;
        }
    }
    if ((index != end) || (decimal_seen && (decimal_count == 0U)) || (minutes >= 60U) ||
        (degrees > maximum_degrees) ||
        ((degrees == maximum_degrees) && ((minutes != 0U) || (fraction != 0U))))
    {
        return false;
    }
    result = (uint64_t)degrees * UINT64_C(1000000);
    minute_microdegrees =
        (((uint64_t)minutes * minute_scale) + fraction) * UINT64_C(1000000) / minute_scale;
    result += minute_microdegrees / UINT64_C(60);
    if (result > UINT32_MAX)
    {
        return false;
    }
    *coordinate = (uint32_t)result;
    return true;
}

/**
 * @brief Validate sentence checksum and extract the RMC field boundaries.
 * @param[in] parser Completed line state with a final NUL byte.
 * @param[out] starts Array of inclusive field starts with capacity AVR_NMEA_MAX_FIELDS.
 * @param[out] ends Array of exclusive field ends with capacity AVR_NMEA_MAX_FIELDS.
 * @param[out] count Number of fields stored.
 * @return True when checksum syntax and XOR value are valid and fields fit in the arrays.
 */
static bool avr_nmea_fields(const avr_nmea_rmc_parser_t *parser, uint8_t *starts, uint8_t *ends,
                            uint8_t *count)
{
    uint8_t star = 0U;
    uint8_t checksum = 0U;
    uint8_t high, low;
    uint8_t field_count = 0U;
    uint8_t field_start = 1U;
    if ((parser == NULL) || (starts == NULL) || (ends == NULL) || (count == NULL) ||
        (parser->length < 6U))
    {
        return false;
    }
    while ((star < parser->length) && (parser->line[star] != '*'))
    {
        star++;
    }
    if ((star >= parser->length) || ((uint8_t)(parser->length - star) != 3U))
    {
        return false;
    }
    if (!avr_nmea_hex_value(parser->line[star + 1U], &high) ||
        !avr_nmea_hex_value(parser->line[star + 2U], &low))
    {
        return false;
    }
    for (uint8_t index = 1U; index < star; index++)
    {
        checksum ^= (uint8_t)parser->line[index];
    }
    if (checksum != (uint8_t)((uint8_t)(high << 4U) | low))
    {
        return false;
    }
    for (uint8_t index = 1U; index <= star; index++)
    {
        if ((index == star) || (parser->line[index] == ','))
        {
            if (field_count >= AVR_NMEA_MAX_FIELDS)
            {
                return false;
            }
            starts[field_count] = field_start;
            ends[field_count] = index;
            field_count++;
            field_start = (uint8_t)(index + 1U);
        }
    }
    *count = field_count;
    return true;
}

/**
 * @brief Parse a complete buffered sentence into an RMC record.
 * @param[in] parser Validated, line-buffered sentence.
 * @param[out] record Caller-owned result updated only for a complete valid RMC record.
 * @return Record, unsupported type or invalid sentence result.
 */
static avr_nmea_result_t avr_nmea_parse_line(const avr_nmea_rmc_parser_t *parser,
                                             avr_nmea_rmc_record_t *record)
{
    uint8_t starts[AVR_NMEA_MAX_FIELDS];
    uint8_t ends[AVR_NMEA_MAX_FIELDS];
    uint8_t count;
    avr_nmea_rmc_record_t candidate = {false, 0U, 0U, '\0', '\0'};
    if (!avr_nmea_fields(parser, starts, ends, &count))
    {
        return AVR_NMEA_INVALID;
    }
    if ((count == 0U) || (ends[0] - starts[0] != 5U) ||
        ((memcmp(&parser->line[starts[0]], "GPRMC", 5U) != 0) &&
         (memcmp(&parser->line[starts[0]], "GNRMC", 5U) != 0)))
    {
        return AVR_NMEA_UNSUPPORTED;
    }
    if (count < 7U)
    {
        return AVR_NMEA_INVALID;
    }
    if (record == NULL)
    {
        return AVR_NMEA_INVALID;
    }
    if ((ends[2] - starts[2] != 1U) ||
        !((parser->line[starts[2]] == 'A') || (parser->line[starts[2]] == 'V')))
    {
        return AVR_NMEA_INVALID;
    }
    candidate.fix_valid = parser->line[starts[2]] == 'A';
    if (!candidate.fix_valid)
    {
        *record = candidate;
        return AVR_NMEA_RECORD;
    }
    if (((ends[4] - starts[4]) != 1U) || ((ends[6] - starts[6]) != 1U) ||
        !((parser->line[starts[4]] == 'N') || (parser->line[starts[4]] == 'S')) ||
        !((parser->line[starts[6]] == 'E') || (parser->line[starts[6]] == 'W')) ||
        !avr_nmea_coordinate(parser->line, starts[3], ends[3], 2U, 90U,
                             &candidate.latitude_microdegrees) ||
        !avr_nmea_coordinate(parser->line, starts[5], ends[5], 3U, 180U,
                             &candidate.longitude_microdegrees))
    {
        return AVR_NMEA_INVALID;
    }
    candidate.latitude_hemisphere = parser->line[starts[4]];
    candidate.longitude_hemisphere = parser->line[starts[6]];
    *record = candidate;
    return AVR_NMEA_RECORD;
}

void avr_nmea_rmc_reset(avr_nmea_rmc_parser_t *parser)
{
    if (parser != NULL)
    {
        parser->length = 0U;
        parser->line[0] = '\0';
        parser->active = false;
        parser->overflow = false;
    }
}

avr_nmea_result_t avr_nmea_rmc_feed(avr_nmea_rmc_parser_t *parser, uint8_t byte,
                                    avr_nmea_rmc_record_t *record)
{
    if (parser == NULL)
    {
        return AVR_NMEA_INVALID;
    }
    if (parser->length > AVR_NMEA_RMC_MAX_LENGTH)
    {
        avr_nmea_rmc_reset(parser);
        return AVR_NMEA_INVALID;
    }
    if (byte == (uint8_t)'$')
    {
        parser->length = 0U;
        parser->line[0] = '\0';
        parser->active = true;
        parser->overflow = false;
        parser->line[parser->length] = '$';
        parser->length++;
        parser->line[parser->length] = '\0';
        return AVR_NMEA_PENDING;
    }
    if (!parser->active)
    {
        return AVR_NMEA_PENDING;
    }
    if (byte == (uint8_t)'\n')
    {
        avr_nmea_result_t result;
        if ((parser->length > 0U) && (parser->line[parser->length - 1U] == '\r'))
        {
            parser->length--;
        }
        parser->line[parser->length] = '\0';
        parser->active = false;
        if (parser->overflow)
        {
            parser->overflow = false;
            parser->length = 0U;
            return AVR_NMEA_INVALID;
        }
        result = avr_nmea_parse_line(parser, record);
        parser->length = 0U;
        return result;
    }
    if (parser->length >= AVR_NMEA_RMC_MAX_LENGTH)
    {
        parser->overflow = true;
        return AVR_NMEA_PENDING;
    }
    if (!parser->overflow)
    {
        parser->line[parser->length] = (char)byte;
        parser->length++;
        parser->line[parser->length] = '\0';
    }
    return AVR_NMEA_PENDING;
}
