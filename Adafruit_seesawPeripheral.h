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
  #define ALL_ADC  0b1111000000110011001111 // pins that have ADC capability
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
#define VALID_ADC ( ALL_GPIO & ~ INVALID_GPIO )




void Adafruit_seesawPeripheral_reset(void) ;
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins);
void receiveEvent(int howMany);
void requestEvent(void);


/****************************************************** global state */

volatile uint8_t i2c_buffer[32];

#if CONFIG_INTERRUPT
  volatile uint32_t g_irqGPIO = 0;
  volatile uint32_t g_irqFlags = 0;
  #define IRQ_PULSE_TICKS 2
#endif

#if CONFIG_ADC
  volatile uint8_t g_adcStatus = 0;
#endif

/****************************************************** code */

// global address
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

  Adafruit_seesawPeripheral_reset();

  SEESAW_DEBUG(F("I2C 0x"));
  SEESAW_DEBUGLN(_i2c_addr, HEX);
  Wire.begin(_i2c_addr);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  return true;
}

void Adafruit_seesawPeripheral_reset(void) {
  uint32_t pins = VALID_GPIO;
  for (uint8_t pin=0; pin<32; pin++) {
    if ((pins >> pin) & 0x1) {
      pinMode(pin, INPUT);
      digitalWrite(pin, 0);
    }
  }
#if CONFIG_INTERRUPT
  g_irqGPIO = 0;
  g_irqFlags = 0;
#endif
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

#endif
