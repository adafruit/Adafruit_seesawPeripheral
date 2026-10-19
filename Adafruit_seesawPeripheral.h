/*!
 *  @file Adafruit_seesawPeripheral.h
 */

#ifndef _ADAFRUIT_SEESAWPERIPHERAL_H
#define _ADAFRUIT_SEESAWPERIPHERAL_H

#include "Adafruit_seesaw.h"
#include "Arduino.h"

/*************** Optional features default to disabled */
#ifndef CONFIG_ADC
#define CONFIG_ADC 0
#endif
#ifndef CONFIG_PWM_16BIT
#define CONFIG_PWM_16BIT 0
#endif
#ifndef CONFIG_PWM
#define CONFIG_PWM 0
#endif
#ifndef CONFIG_NEOPIXEL
#define CONFIG_NEOPIXEL 0
#endif
#if CONFIG_NEOPIXEL && !defined(CONFIG_NEOPIXEL_BUF_MAX)
#define CONFIG_NEOPIXEL_BUF_MAX 192 // Bytes; sketches may override.
#endif
#ifndef CONFIG_UART
#define CONFIG_UART 0
#endif
#ifndef CONFIG_ENCODER
#define CONFIG_ENCODER 0
#endif
#ifndef CONFIG_FHT
#define CONFIG_FHT 0
#endif

#if defined(ARDUINO_ARCH_STM32)
#include "Adafruit_seesawPeripheral_stm32_config.h"
#else
#include "Adafruit_seesawPeripheral_avr_config.h"
#endif

void foo(void);


#if !defined(CONFIG_EEPROM)
#if defined(ARDUINO_ARCH_STM32)
#define CONFIG_EEPROM 0
#else
#define CONFIG_EEPROM 1
#endif
#endif

#if CONFIG_EEPROM
#define EEPROM_I2C_ADDR (Adafruit_seesawPeripheral_eepromLength() - 1)
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

#define CONFIG_VERSION (((uint32_t)PRODUCT_CODE << 16) | DATE_CODE)

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

#if defined(ARDUINO_ARCH_STM32)
#define SEESAW_I2C_PIN_MASK                                                    \
  ((1UL << CONFIG_I2C_SDA_PIN) | (1UL << CONFIG_I2C_SCL_PIN))
#else
#define SEESAW_I2C_PIN_MASK ((1UL << SDA) | (1UL << SCL))
#endif

#define INVALID_GPIO                                                           \
  (SEESAW_I2C_PIN_MASK | ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_RXD) |     \
   ((uint32_t)CONFIG_UART_DEBUG << UART_DEBUG_TXD) |                           \
   ((uint32_t)CONFIG_INTERRUPT << CONFIG_INTERRUPT_PIN) |                      \
   ((uint32_t)CONFIG_ADDR_0 << CONFIG_ADDR_0_PIN) |                            \
   ((uint32_t)CONFIG_ADDR_1 << CONFIG_ADDR_1_PIN) |                            \
   ((uint32_t)CONFIG_ADDR_2 << CONFIG_ADDR_2_PIN) |                            \
   ((uint32_t)CONFIG_ADDR_3 << CONFIG_ADDR_3_PIN) | 0)

#define VALID_GPIO (ALL_GPIO & ~INVALID_GPIO)
#define VALID_ADC (ALL_ADC & VALID_GPIO)
#define VALID_PWM (ALL_PWM & VALID_GPIO)

void Adafruit_seesawPeripheral_reset(void);
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins);
// Keep the decoder separate under LTO. Inlining it can trigger short
// interrupt-vector jump overflows with the 8 KB tinyAVR linker.
void Adafruit_seesawPeripheral_processCommand(const volatile uint8_t *packet,
                                              int howMany)
    __attribute__((noinline));
void Adafruit_seesawPeripheral_processRequest(uint8_t base_cmd,
                                              uint8_t module_cmd);
void Adafruit_seesawPeripheral_run(void);
void Adafruit_seesawPeripheral_changedGPIO(void);

#if defined(ARDUINO_ARCH_STM32)
#include "Adafruit_seesawPeripheral_stm32_i2c.h"
#else
#include "Adafruit_seesawPeripheral_i2c.h"
#endif

/****************************************************** global state */

volatile uint32_t g_bufferedBulkGPIORead =
    0; ///< GPIO snapshot for the next read.

#if CONFIG_INTERRUPT
volatile uint32_t g_irqGPIO = 0;
volatile uint32_t g_irqFlags = 0;
volatile uint8_t IRQ_debounce_cntr = 0;
#define IRQ_DEBOUNCE_TICKS 3 // in millis
#endif

#if CONFIG_ADC
volatile uint16_t g_bufferedADCRead = 0; ///< ADC result for the next read.
volatile uint8_t g_adcStatus = 0;
#endif
#if (CONFIG_PWM | CONFIG_PWM_16BIT)
volatile uint8_t g_pwmStatus = 0;
#endif
#if CONFIG_NEOPIXEL
#if !defined(ARDUINO_ARCH_STM32)
volatile uint8_t g_neopixel_buf[CONFIG_NEOPIXEL_BUF_MAX];
#endif
volatile uint16_t g_neopixel_bufsize = 0;
volatile uint8_t g_neopixel_pin = 0;
#endif
#if CONFIG_UART
volatile uint8_t g_uart_buf[CONFIG_UART_BUF_MAX];
volatile uint8_t g_uart_status = 0;
volatile uint8_t g_uart_inten = 0;
volatile uint32_t g_uart_baud = 9600;
volatile uint8_t g_uart_tx_len = 0;
#endif

#if CONFIG_ENCODER

#define ENCODER_FLAG_FORW_EDGE1 0x01
#define ENCODER_FLAG_BACK_EDGE1 0x02
#define ENCODER_FLAG_FORW_EDGE2 0x04
#define ENCODER_FLAG_BACK_EDGE2 0x08
#define ENCODER_FLAG_MIDSTEP    0x10

#define BIT_IS_SET(x,b) (((x)&(1UL << b)) != 0)
#define BIT_IS_CLEAR(x,b) (((x)&(1UL << b)) == 0)

#define ENCODER0_INPUT_MASK ((1UL << CONFIG_ENCODER0_A_PIN) | (1UL << CONFIG_ENCODER0_B_PIN))

#ifdef CONFIG_ENCODER1_A_PIN
#define ENCODER1_INPUT_MASK ((1UL << CONFIG_ENCODER1_A_PIN) | (1UL << CONFIG_ENCODER1_B_PIN))
#else
#define ENCODER1_INPUT_MASK 0
#endif
#ifdef CONFIG_ENCODER2_A_PIN
#define ENCODER2_INPUT_MASK ((1UL << CONFIG_ENCODER2_A_PIN) | (1UL << CONFIG_ENCODER2_B_PIN))
#else
#define ENCODER2_INPUT_MASK 0
#endif
#ifdef CONFIG_ENCODER3_A_PIN
#define ENCODER3_INPUT_MASK ((1UL << CONFIG_ENCODER3_A_PIN) | (1UL << CONFIG_ENCODER3_B_PIN))
#else
#define ENCODER3_INPUT_MASK 0
#endif

#ifndef CONFIG_ENCODER_2TICKS
#define CONFIG_ENCODER_2TICKS 0

#endif

volatile int32_t g_enc_value[CONFIG_NUM_ENCODERS];
volatile int32_t g_enc_delta[CONFIG_NUM_ENCODERS];
volatile uint8_t g_enc_prev_pos[CONFIG_NUM_ENCODERS];
volatile uint8_t g_enc_flags[CONFIG_NUM_ENCODERS];

#endif

#if CONFIG_ENCODER
/*! Decode an active-low A/B phase with the same detent rules on all targets. */
void Adafruit_seesawPeripheral_updateEncoder(uint8_t encodernum,
                                             uint8_t enc_cur_pos) {
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
  // if any rotation at all
  if (enc_cur_pos != g_enc_prev_pos[encodernum]) {
    SEESAW_DEBUG(F("Enc0 CurPos 0x"));
    SEESAW_DEBUGLN(enc_cur_pos, HEX);

    if (g_enc_prev_pos[encodernum] == 0x00) {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        g_enc_flags[encodernum] |= ENCODER_FLAG_FORW_EDGE1;
      }
      else if (enc_cur_pos == 0x02) {
        g_enc_flags[encodernum] |= ENCODER_FLAG_BACK_EDGE1;
      }
    }

    if (g_enc_prev_pos[encodernum] == 0x03) {
      // this is the second edge
      if (enc_cur_pos == 0x02) {
        g_enc_flags[encodernum] |= ENCODER_FLAG_FORW_EDGE2;
      }
      else if (enc_cur_pos == 0x01) {
        g_enc_flags[encodernum] |= ENCODER_FLAG_BACK_EDGE2;
      }
    }

    if ((enc_cur_pos == 0x03) && ! CONFIG_ENCODER_2TICKS) {
      // this is when the encoder is in the middle of a "step" of a 4-tick encoder
      g_enc_flags[encodernum] |= ENCODER_FLAG_MIDSTEP;
    }

    if ((enc_cur_pos == 0x00) || ((enc_cur_pos == 0x03) && CONFIG_ENCODER_2TICKS))
    {
      // this is when the encoder is in a 'rest' state

      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if ((g_enc_flags[encodernum] & ENCODER_FLAG_FORW_EDGE1) &&
          (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_FORW_EDGE2 | ENCODER_FLAG_MIDSTEP)))) {
        SEESAW_DEBUG(F("+1"));
        enc_action = 1;
      }
      else if ((g_enc_flags[encodernum] & ENCODER_FLAG_FORW_EDGE2) &&
               (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_FORW_EDGE1 | ENCODER_FLAG_MIDSTEP)))) {
        SEESAW_DEBUG(F("+2"));
        enc_action = 1;
      }
      else if ((g_enc_flags[encodernum] & ENCODER_FLAG_BACK_EDGE1) &&
               (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_BACK_EDGE2 | ENCODER_FLAG_MIDSTEP)))) {
        SEESAW_DEBUG(F("-1"));
        enc_action = -1;
      }
      else if ((g_enc_flags[encodernum] & ENCODER_FLAG_BACK_EDGE2) &&
               (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_BACK_EDGE1 | ENCODER_FLAG_MIDSTEP)))) {
        SEESAW_DEBUG(F("-2"));
        enc_action = -1;
      }

      g_enc_flags[encodernum] = 0; // reset for next time
    }
  }

  g_enc_prev_pos[encodernum] = enc_cur_pos;

  if(enc_action != 0){
#if defined(ARDUINO_ARCH_STM32)
    // Do not lose a read-to-clear I2C callback between the delta load/store.
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();
#endif
    g_enc_value[encodernum] += enc_action;
    g_enc_delta[encodernum] += enc_action;
#if defined(ARDUINO_ARCH_STM32)
    __set_PRIMASK(savedMask);
#endif
  }
}
#endif

/****************************************************** code */

void Adafruit_seesawPeripheral_setDatecode(void) {
  const char *date = __DATE__;
  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  uint8_t month = 1;
  for (; month <= 12; month++) {
    uint8_t pos = (month - 1) * 3;
    if (date[0] == months[pos] && date[1] == months[pos + 1] &&
        date[2] == months[pos + 2]) {
      break;
    }
  }
  uint8_t day = date[5] - '0';
  if (date[4] != ' ') {
    day += (date[4] - '0') * 10;
  }
  uint16_t year = 0;
  for (uint8_t i = 7; i < 11; i++) {
    year = year * 10 + date[i] - '0';
  }
  // Preserve the day/month/year layout and legacy 6-bit year mask.
  DATE_CODE =
      ((uint16_t)day << 11) | ((uint16_t)month << 7) | ((year - 2000) & 0x3F);
}

#if defined(ARDUINO_ARCH_STM32)
#include "Adafruit_seesawPeripheral_stm32.h"
#else
#include "Adafruit_seesawPeripheral_avr.h"

bool Adafruit_seesawPeripheral_begin(void) {
  SEESAW_DEBUG(F("All GPIO: "));
  SEESAW_DEBUGLN(ALL_GPIO, HEX);
  SEESAW_DEBUG(F("Invalid: "));
  SEESAW_DEBUGLN(INVALID_GPIO, HEX);
  SEESAW_DEBUG(F("Valid: "));
  SEESAW_DEBUGLN(VALID_GPIO, HEX);

#ifdef CONFIG_INTERRUPT
  Adafruit_seesawPeripheral_clearIRQ();
#endif

  Adafruit_seesawPeripheral_reset();

  // Register callbacks after reset has started the transport.
  Adafruit_seesawPeripheral_attachI2CCallbacks();
  return true;
}

void Adafruit_seesawPeripheral_reset(void) {
  Adafruit_seesawPeripheral_setDatecode();

  Adafruit_seesawPeripheral_enableInterrupts(false);

  SEESAW_DEBUGLN(F("Wire end"));
  Adafruit_seesawPeripheral_endI2C();

  // Not referenced after starting I2C, so this is now local
  uint8_t _i2c_addr = CONFIG_I2C_PERIPH_ADDR;

#if CONFIG_EEPROM
  _i2c_addr = Adafruit_seesawPeripheral_readEEPROM(EEPROM_I2C_ADDR);
  SEESAW_DEBUG(F("EEaddr: 0x"));
  SEESAW_DEBUGLN(_i2c_addr, HEX);
  if (_i2c_addr > 0x7F) {
    _i2c_addr = CONFIG_I2C_PERIPH_ADDR;
  }
#endif

  _i2c_addr = Adafruit_seesawPeripheral_applyAddressStraps(_i2c_addr);

  SEESAW_DEBUG(F("I2C 0x"));
  SEESAW_DEBUGLN(_i2c_addr, HEX);

  Adafruit_seesawPeripheral_initializeGPIO();
#if CONFIG_INTERRUPT
  g_irqGPIO = 0;
  g_irqFlags = 0;
#endif
#if CONFIG_ADC
  g_adcStatus = 0;
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  Adafruit_seesawPeripheral_initializePWM();
#endif
#if CONFIG_NEOPIXEL
  for (uint16_t i = 0; i < CONFIG_NEOPIXEL_BUF_MAX; i++) {
    g_neopixel_buf[i] = 0;
  }
  g_neopixel_bufsize = 0;
#endif
#if CONFIG_ENCODER
  Adafruit_seesawPeripheral_initializeEncoderPins();

  for (uint8_t encodernum=0; encodernum<CONFIG_NUM_ENCODERS; encodernum++) {
    g_enc_value[encodernum] = 0;
    g_enc_delta[encodernum] = 0;
    g_enc_prev_pos[encodernum] = 0;
    g_enc_flags[encodernum] = 0;
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
  Adafruit_seesawPeripheral_initializeSpectrum();
#endif

#if CONFIG_UART
  Adafruit_seesawPeripheral_initializeUART();
#endif

  Adafruit_seesawPeripheral_beginI2C(_i2c_addr);
  Adafruit_seesawPeripheral_enableInterrupts(true);
}

#endif // ARDUINO_ARCH_STM32

/**
 * @brief Enable or disable change interrupts for selected valid GPIO pins.
 * @param mask GPIO bits to change; invalid pins are ignored.
 * @param enabled True to enable interrupts, false to disable them.
 */
void Adafruit_seesawPeripheral_gpioInterrupt(uint32_t mask, bool enabled) {
#if !CONFIG_INTERRUPT
  (void)enabled;
#endif
  mask &= VALID_GPIO;
  for (uint8_t pin = 0; pin < 32; pin++) {
    if ((mask >> pin) & 0x1) {
      SEESAW_DEBUG(F("Set pin "));
      SEESAW_DEBUG(pin);

#if CONFIG_INTERRUPT
      Adafruit_seesawPeripheral_setGPIOInterrupt(pin, enabled);
#endif
    }
  }
}

#if CONFIG_ENCODER && !defined(ARDUINO_ARCH_STM32)
/**
 * @brief Look up the input pin mask for an encoder.
 * @param encoder Encoder index, from 0 through 3.
 * @return Input pin mask, or zero for an unknown encoder index.
 */
uint32_t Adafruit_seesawPeripheral_encoderInputMask(uint8_t encoder) {
  switch (encoder) {
  case 0:
    return ENCODER0_INPUT_MASK;
  case 1:
    return ENCODER1_INPUT_MASK;
  case 2:
    return ENCODER2_INPUT_MASK;
  case 3:
    return ENCODER3_INPUT_MASK;
  }
  return 0;
}

/**
 * @brief Enable or disable GPIO change reporting for an encoder's inputs.
 * @param encoder Encoder index, from 0 through 3.
 * @param enabled True to enable interrupts, false to disable them.
 */
void Adafruit_seesawPeripheral_setEncoderInterrupt(uint8_t encoder,
                                                   bool enabled) {
  uint32_t mask = Adafruit_seesawPeripheral_encoderInputMask(encoder);
  if (enabled) {
    g_irqGPIO |= mask;
  } else {
    g_irqGPIO &= ~mask;
  }
}
#endif


#if CONFIG_UART
/**
 * @brief Copy a UART write into the buffer drained by the main loop.
 * @param data Payload bytes to transmit.
 * @param length Payload length; oversized writes clear the pending length.
 */
void Adafruit_seesawPeripheral_queueUARTData(const volatile uint8_t *data,
                                             uint8_t length) {
  g_uart_tx_len = length;
  if (g_uart_tx_len <= CONFIG_UART_BUF_MAX) {
    for (uint8_t i = 0; i < g_uart_tx_len; i++) {
      g_uart_buf[i] = data[i];
    }
  } else {
    g_uart_tx_len = 0;
  }
}

#endif

#if !defined(ARDUINO_ARCH_STM32)
#include "Adafruit_seesawPeripheral_main.h"
#endif
#include "Adafruit_seesawPeripheral_receive.h"
#include "Adafruit_seesawPeripheral_request.h"
#endif
