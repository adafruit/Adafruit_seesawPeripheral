/*!
 *  @file Adafruit_seesawPeripheral.h
 */

#ifndef _ADAFRUIT_SEESAWPERIPHERAL_H
#define _ADAFRUIT_SEESAWPERIPHERAL_H

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_seesaw.h"

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


#define DATE_CODE 1234 // FIXME

#define CONFIG_VERSION (uint32_t)( ( (uint32_t)PRODUCT_CODE << 16 ) | ( (uint16_t)DATE_CODE & 0x0000FFFF) )

/********************** Hardcoded chip configration */

#ifdef ARDUINO_AVR_ATtiny817
  #define UART_DEBUG_RXD 8
  #define UART_DEBUG_TXD 9
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


/********************** Available/taken GPIO configuration macros */

#ifdef ARDUINO_AVR_ATtiny817
  #define ALL_GPIO 0x0FFFFFUL       // this is chip dependant, for 817 we have 20 GPIO avail
#endif

#define INVALID_GPIO ((1UL << SDA) | (1UL << SCL) | \
                      ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_RXD) | \
                      ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_TXD)   |   \
                      ((uint32_t)CONFIG_INTERRUPT << CONFIG_INTERRUPT_PIN) | \
                      ((uint32_t)CONFIG_ADDR_0 << CONFIG_ADDR_0_PIN) | \
                      ((uint32_t)CONFIG_ADDR_1 << CONFIG_ADDR_1_PIN) | \
                      ((uint32_t)CONFIG_ADDR_2 << CONFIG_ADDR_2_PIN) | \
                      0)

#define VALID_GPIO ( ALL_GPIO & ~ INVALID_GPIO )



/****************************************************** code */


uint8_t _i2c_addr = CONFIG_I2C_PERIPH_ADDR;

bool Adafruit_seesawPeripheral_begin(void) {
  
  SEESAW_DEBUG("All GPIO: "); 
  SEESAW_DEBUGLN(ALL_GPIO, HEX);
  SEESAW_DEBUG("Invalid: "); 
  SEESAW_DEBUGLN(INVALID_GPIO, HEX);
  SEESAW_DEBUG("Valid: "); 
  SEESAW_DEBUGLN(VALID_GPIO, HEX);


#ifdef CONFIG_INTERRUPT
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT);
  digitalWrite(CONFIG_INTERRUPT_PIN, LOW); 
#endif

#ifdef CONFIG_ADDR_0
  pinMode(CONFIG_ADDR_0_PIN, INPUT_PULLUP);
  if (!digitalRead(CONFIG_ADDR_0_PIN))
    _i2c_addr += 1;
#endif
#ifdef CONFIG_ADDR_1
  pinMode(CONFIG_ADDR_1_PIN, INPUT_PULLUP);
  if (!digitalRead(CONFIG_ADDR_1_PIN))
    _i2c_addr += 2;
#endif
#ifdef CONFIG_ADDR_2
  pinMode(CONFIG_ADDR_2_PIN, INPUT_PULLUP);
  if (!digitalRead(CONFIG_ADDR_2_PIN))
    _i2c_addr += 4;
#endif


  Wire.begin(_i2c_addr);

  return true;
}


#endif
