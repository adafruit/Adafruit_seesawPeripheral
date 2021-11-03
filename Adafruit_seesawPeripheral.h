/*!
 *  @file Adafruit_seesawPeripheral.h
 */

#ifndef _ADAFRUIT_SEESAWPERIPHERAL_H
#define _ADAFRUIT_SEESAWPERIPHERAL_H

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_seesaw.h"

void foo(void);

#if CONFIG_NEOPIXEL && defined(MEGATINYCORE)
  #include "Adafruit_seesawPeripheral_tinyneopixel.h"
#endif

#if !defined(CONFIG_EEPROM)
  #define CONFIG_EEPROM  1
#endif

#if CONFIG_EEPROM
  #include <EEPROM.h>
  #define EEPROM_I2C_ADDR (EEPROM.length()-1)
  bool _i2c_started = false;
#endif

/*************** UART debugging */
#if !defined(CONFIG_UART_DEBUG)
  #define CONFIG_UART_DEBUG 0
#endif

#if (CONFIG_UART_DEBUG == 1)
  #define SEESAW_DEBUG(...) Serial.print(__VA_ARGS__)
  #define SEESAW_DEBUGLN(...) Serial.println(__VA_ARGS__)
#elif (CONFIG_UART_DEBUG == 0)
  #define SEESAW_DEBUG(...)
  #define SEESAW_DEBUGLN(...)
#else
  #error("CONFIG_UART_DEBUG must be 0 or 1")
#endif

/*************** Interrupt Pin */
#if defined(CONFIG_INTERRUPT_PIN)
  #define CONFIG_INTERRUPT 1
#else
  #define CONFIG_INTERRUPT 0
  #define CONFIG_INTERRUPT_PIN 0
#endif
#if defined(USE_PINCHANGE_INTERRUPT)
  #define USE_PINCHANGE_INTERRUPT 1
#else
  #define USE_PINCHANGE_INTERRUPT 0
#endif

uint16_t DATE_CODE = 0;

#define CONFIG_VERSION (uint32_t)( ( (uint32_t)PRODUCT_CODE << 16 ) | ( (uint16_t)DATE_CODE & 0x0000FFFF) )

/********************** Hardcoded chip configration */

#if defined(ARDUINO_AVR_ATtiny817) || defined(ARDUINO_AVR_ATtiny807)
  #define UART_DEBUG_RXD 8
  #define UART_DEBUG_TXD 9
#endif
#if defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny806)
  #define UART_DEBUG_RXD 6
  #define UART_DEBUG_TXD 7
#endif

#ifdef CONFIG_ADDR_INVERTED
  #undef CONFIG_ADDR_INVERTED
  #define CONFIG_ADDR_INVERTED 1
#else
  #define CONFIG_ADDR_INVERTED 0
#endif

#ifdef CONFIG_ADDR_0_PIN
  #define CONFIG_ADDR_0 1
#else
  #define CONFIG_ADDR_0 0
  #define CONFIG_ADDR_0_PIN 0
#endif
#ifdef CONFIG_ADDR_1_PIN
  #define CONFIG_ADDR_1 1
#else
  #define CONFIG_ADDR_1 0
  #define CONFIG_ADDR_1_PIN 0
#endif
#ifdef CONFIG_ADDR_2_PIN
  #define CONFIG_ADDR_2 1
#else
  #define CONFIG_ADDR_2 0
  #define CONFIG_ADDR_2_PIN 0
#endif
#ifdef CONFIG_ADDR_3_PIN
  #define CONFIG_ADDR_3 1
#else
  #define CONFIG_ADDR_3 0
  #define CONFIG_ADDR_3_PIN 0
#endif

/********************** Available/taken GPIO configuration macros */

#if defined(ARDUINO_AVR_ATtiny817) || defined(ARDUINO_AVR_ATtiny807)
  #define ALL_GPIO 0x1FFFFFUL  // this is chip dependant, for 817 we have 21 GPIO avail (0~20 inc)
  #define ALL_ADC  0b1111000000110011001111 // pins that have ADC capability
  #define ALL_PWM  ((1UL << 0) | (1UL << 1) | (1UL << 9) | (1UL << 10) | \
                    (1UL << 11) | (1UL << 12) | (1UL << 13) | (1UL << 10))
#endif

#if defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny806)
  #define ALL_GPIO 0x01FFFFUL  // this is chip dependant, for 816 we have 17 GPIO avail
  #define ALL_ADC  0b11100001100111111 // pins that have ADC capability
  #define ALL_PWM  ((1UL << 0) | (1UL << 1) | (1UL << 7) | (1UL << 8) | \
                    (1UL << 9) | (1UL << 10) | (1UL << 11) | (1UL << 16))
#endif

#define INVALID_GPIO ((1UL << SDA) | (1UL << SCL) | \
                      ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_RXD) | \
                      ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_TXD)   |   \
                      ((uint32_t)CONFIG_INTERRUPT << CONFIG_INTERRUPT_PIN) | \
                      ((uint32_t)CONFIG_ADDR_0 << CONFIG_ADDR_0_PIN) | \
                      ((uint32_t)CONFIG_ADDR_1 << CONFIG_ADDR_1_PIN) | \
                      ((uint32_t)CONFIG_ADDR_2 << CONFIG_ADDR_2_PIN) | \
                      ((uint32_t)CONFIG_ADDR_3 << CONFIG_ADDR_3_PIN) | \
                      0)

#define VALID_GPIO ( ALL_GPIO & ~ INVALID_GPIO )
#define VALID_ADC ( ALL_ADC & VALID_GPIO )
#define VALID_PWM ( ALL_PWM & VALID_GPIO )


void Adafruit_seesawPeripheral_reset(void) ;
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins);
void receiveEvent(int howMany);
void requestEvent(void);
void Adafruit_seesawPeripheral_run(void);
void Adafruit_seesawPeripheral_changedGPIO(void);

/****************************************************** global state */

volatile uint8_t i2c_buffer[32];

#if CONFIG_INTERRUPT
  volatile uint32_t g_irqGPIO = 0;
  volatile uint32_t g_irqFlags = 0;
  volatile uint8_t IRQ_pulse_cntr = 0;
  #define IRQ_PULSE_TICKS 10 // in millis
#endif

#if CONFIG_ADC
  volatile uint8_t g_adcStatus = 0;
#endif
#if CONFIG_PWM
  volatile uint8_t g_pwmStatus = 0;
#endif
#if CONFIG_NEOPIXEL 
  volatile uint8_t g_neopixel_buf[CONFIG_NEOPIXEL_BUF_MAX];
  volatile uint8_t g_neopixel_bufsize = 0;
  volatile uint8_t g_neopixel_pin = 0;
#endif

/****************************************************** code */

// global address
uint8_t _i2c_addr = CONFIG_I2C_PERIPH_ADDR;

void Adafruit_seesawPeripheral_setDatecode(void) {

  char buf[12];
  char *bufp = buf;
  int month = 0, day = 0, year = 2000;
  static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

  strncpy(buf, __DATE__, 11);
  buf[11] = 0;

  bufp[3] = 0;
  month = (strstr(month_names, bufp)-month_names)/3 + 1;

  bufp += 4;
  bufp[2] = 0;
  day = atoi(bufp);

  bufp += 3;
  year = atoi(bufp);

  DATE_CODE = day & 0x1F; // top 5 bits are day of month

  DATE_CODE <<= 4;
  DATE_CODE |= month & 0xF; // middle 4 bits are month

  DATE_CODE <<= 7;
  DATE_CODE |= (year - 2000) & 0x3F; // bottom 7 bits are year
}


bool Adafruit_seesawPeripheral_begin(void) {
  SEESAW_DEBUG(F("All GPIO: ")); 
  SEESAW_DEBUGLN(ALL_GPIO, HEX);
  SEESAW_DEBUG(F("Invalid: ")); 
  SEESAW_DEBUGLN(INVALID_GPIO, HEX);
  SEESAW_DEBUG(F("Valid: ")); 
  SEESAW_DEBUGLN(VALID_GPIO, HEX);


#ifdef CONFIG_INTERRUPT
  pinMode(CONFIG_INTERRUPT_PIN, INPUT_PULLUP); // open-drainish
#endif

  Adafruit_seesawPeripheral_reset();

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  return true;
}

void Adafruit_seesawPeripheral_reset(void) {
  Adafruit_seesawPeripheral_setDatecode();

  cli();

  SEESAW_DEBUGLN(F("Wire end"));
  Wire.end();

#if CONFIG_EEPROM
  _i2c_addr = EEPROM.read(EEPROM_I2C_ADDR);
  SEESAW_DEBUG(F("EEaddr: 0x"));
  SEESAW_DEBUGLN(_i2c_addr, HEX);
  if (_i2c_addr > 0x7F) {
    _i2c_addr = CONFIG_I2C_PERIPH_ADDR;
  }
#endif

#if CONFIG_ADDR_0
  pinMode(CONFIG_ADDR_0_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_0_PIN) == CONFIG_ADDR_INVERTED)
    _i2c_addr += 1;
#endif
#if CONFIG_ADDR_1
  pinMode(CONFIG_ADDR_1_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_1_PIN) == CONFIG_ADDR_INVERTED)
    _i2c_addr += 2;
#endif
#if CONFIG_ADDR_2
  pinMode(CONFIG_ADDR_2_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_2_PIN) == CONFIG_ADDR_INVERTED)
    _i2c_addr += 4;
#endif
#if CONFIG_ADDR_3
  pinMode(CONFIG_ADDR_3_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_3_PIN) == CONFIG_ADDR_INVERTED)
    _i2c_addr += 8;
#endif

  SEESAW_DEBUG(F("I2C 0x"));
  SEESAW_DEBUGLN(_i2c_addr, HEX);

  uint32_t pins = VALID_GPIO;
  for (uint8_t pin=0; pin<32; pin++) {
    if ((pins >> pin) & 0x1) {
      pinMode(pin, INPUT);
      digitalWrite(pin, 0);
#if USE_PINCHANGE_INTERRUPT
      detachInterrupt(digitalPinToInterrupt(pin));
#endif
    }
  }
#if CONFIG_INTERRUPT
  g_irqGPIO = 0;
  g_irqFlags = 0;
#endif
#if CONFIG_ADC
  g_adcStatus = 0;
#endif
#if CONFIG_PWM
  g_pwmStatus = 0;
#endif
#if CONFIG_NEOPIXEL
  for (uint16_t i=0; i<CONFIG_NEOPIXEL_BUF_MAX; i++) {
    g_neopixel_buf[i] = 0;
  }
  g_neopixel_bufsize = 0;
#endif

  Wire.begin(_i2c_addr);
  _i2c_started = true;
  sei();
}


uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins=VALID_GPIO) {
  uint32_t temp = 0;

  for (uint8_t pin=0; pin<32; pin++) {
    if ((validpins >> pin) & 0x1) {
      if (digitalRead(pin)) {
        temp |= 1UL << pin;
      }
    }
  }
  return temp;
}

void Adafruit_seesawPeripheral_write32(uint32_t value) {
  uint8_t buff[4];
  buff[0] = value >> 24;
  buff[1] = value >> 16;
  buff[2] = value >> 8;
  buff[3] = value;
  Wire.write(buff, 4);
  return;
}


#include "Adafruit_seesawPeripheral_receive.h"
#include "Adafruit_seesawPeripheral_request.h"
#include "Adafruit_seesawPeripheral_main.h"
#endif
