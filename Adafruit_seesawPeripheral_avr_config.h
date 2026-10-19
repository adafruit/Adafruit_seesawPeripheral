/*!
 * @file Adafruit_seesawPeripheral_avr_config.h
 * @brief AVR chip identities and hardware pin configuration.
 */
#ifndef ADAFRUIT_SEESAW_PERIPHERAL_AVR_CONFIG_H
#define ADAFRUIT_SEESAW_PERIPHERAL_AVR_CONFIG_H

#if defined(ARDUINO_AVR_ATtiny806)
#define SEESAW_HW_ID 0x84
#elif defined(ARDUINO_AVR_ATtiny807)
#define SEESAW_HW_ID 0x85
#elif defined(ARDUINO_AVR_ATtiny816)
#define SEESAW_HW_ID 0x86
#elif defined(ARDUINO_AVR_ATtiny817)
#define SEESAW_HW_ID 0x87
#elif defined(ARDUINO_AVR_ATtiny1616)
#define SEESAW_HW_ID 0x88
#elif defined(ARDUINO_AVR_ATtiny1617)
#define SEESAW_HW_ID 0x89
#else
#error "Unsupported chip variant selected"
#endif

#if defined(ARDUINO_AVR_ATtiny817) || defined(ARDUINO_AVR_ATtiny807) ||        \
    defined(ARDUINO_AVR_ATtiny1617) || defined(ARDUINO_AVR_ATtiny1607) ||      \
    defined(ARDUINO_AVR_ATtiny427) || defined(ARDUINO_AVR_ATtiny827) ||        \
    defined(ARDUINO_AVR_ATtiny3217)
#define UART_DEBUG_RXD 8
#define UART_DEBUG_TXD 9
#endif
#if defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny806) ||        \
    defined(ARDUINO_AVR_ATtiny1616) || defined(ARDUINO_AVR_ATtiny1606) ||      \
    defined(ARDUINO_AVR_ATtiny3216)
#define UART_DEBUG_RXD 6
#define UART_DEBUG_TXD 7
#endif

#endif
