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
#define ALL_GPIO                                                               \
  0x1FFFFFUL // this is chip dependant, for 817 we have 21 GPIO avail (0~20 inc)
#define ALL_ADC 0b1111000000110011001111 // pins that have ADC capability
#if CONFIG_PWM_16BIT
#define ALL_PWM ((1UL << 6) | (1UL << 7) | (1UL << 8)) // alternate TCA0 WOx
#else
#define ALL_PWM                                                                \
  ((1UL << 0) | (1UL << 1) | (1UL << 9) | (1UL << 10) | (1UL << 11) |          \
   (1UL << 12) | (1UL << 13) | (1UL << 10))
#endif
#define PWM_WO_OFFSET (6)
#endif
#if defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny806) ||        \
    defined(ARDUINO_AVR_ATtiny1616) || defined(ARDUINO_AVR_ATtiny1606) ||      \
    defined(ARDUINO_AVR_ATtiny3216)
#define UART_DEBUG_RXD 6
#define UART_DEBUG_TXD 7
#define ALL_GPIO                                                               \
  0x01FFFFUL // this is chip dependant, for 816 we have 17 GPIO avail
#define ALL_ADC 0b11100001100111111 // pins that have ADC capability
#if CONFIG_PWM_16BIT
#define ALL_PWM ((1UL << 4) | (1UL << 5) | (1UL << 6)) // alternate TCA0 WOx
#else
#define ALL_PWM                                                                \
  ((1UL << 0) | (1UL << 1) | (1UL << 7) | (1UL << 8) | (1UL << 9) |            \
   (1UL << 10) | (1UL << 11) | (1UL << 16))
#endif
#define PWM_WO_OFFSET (4)
#endif

#endif
