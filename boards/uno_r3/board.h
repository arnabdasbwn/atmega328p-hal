/**
 * @file board.h
 * @brief Arduino Uno R3 board constants.
 * @details Defines header pin aliases and reference memory limits. This course targets the
 * ATmega328P at 16 MHz with a 512-byte Optiboot reservation; other bootloader sizes require a
 * different flash limit.
 * @copyright Copyright (c) 2026 Arnab Kumar Das. CC-BY-SA-3.0.
 */
/* SPDX-License-Identifier: CC-BY-SA-3.0 */
#ifndef AVR_UNO_R3_BOARD_H
#define AVR_UNO_R3_BOARD_H
/** @brief Onboard LED L is connected to Uno D13/PB5. */
#define AVR_BOARD_LED 13U
/** @brief First analog input A0 has Uno digital alias 14. */
#define AVR_BOARD_A0 14U
/** @brief TWI SDA is A4/PC4, digital alias 18. */
#define AVR_BOARD_SDA 18U
/** @brief TWI SCL is A5/PC5, digital alias 19. */
#define AVR_BOARD_SCL 19U
/** @brief Application Flash limit in bytes for the reference 512-byte bootloader. */
#define AVR_BOARD_FLASH_LIMIT 32256UL
/** @brief Physical SRAM capacity in bytes; globals and stack share this space. */
#define AVR_BOARD_SRAM_BYTES 2048U
#endif
