# ATmega328P Teaching HAL

A C99 peripheral HAL and device-driver library for the Arduino Uno R3, using its ATmega328P at 16 MHz. Public interfaces are declared in headers; implementations and mutable state live in C source files.

## Repository Layout

- `include/avr_hal/` contains public peripheral headers and API contracts.
- `src/` contains GPIO, ADC, EEPROM, SPI, timer, TWI, USART, common and system implementations.
- `lib/drivers/` contains display, sensor, RFID, radio, serial-protocol and SD-card drivers.
- `lib/filesystems/` contains bounded, read-only FAT16 support.
- `lib/interrupts/` contains optional timer-tick and UART receive-queue interrupt owners. These files are selected by applications when needed.
- `boards/uno_r3/` contains Uno R3 clock, pin and memory definitions.
- `cmake/` contains AVR GCC and XC8 toolchain files.

## Build The Static Library

Set the compiler paths before configuring: `AVR_GCC` for AVR GCC, or `XC8_CC` and `AVR_DFP` for XC8. From this directory, for AVR GCC: 

```powershell
cmake -S . -B build/gcc -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc.cmake
cmake --build build/gcc
```

This builds the `avr_hal` static library and does not upload firmware. Set `F_CPU=16000000UL` for the Uno R3. Assign one owner to each interrupt vector and peripheral resource. Initializers do not enable global interrupts unless their API contract says otherwise.

Read the public headers for pin mappings, bounds, buffer ownership, blocking behavior, timeouts and status results. Lab projects select optional interrupt sources explicitly.

## License And Attribution

HAL implementation and course material follow the terms in `LICENSE.md`. Third-party components retain their own notices.
