/*!
 *  @file Adafruit_seesawPeripheral.h
 */

#ifndef _ADAFRUIT_SEESAWPERIPHERAL_H
#define _ADAFRUIT_SEESAWPERIPHERAL_H

#include "Adafruit_seesaw.h"
#include "Arduino.h"
#include <Wire.h>
#include "wiring_private.h"
#include "pins_arduino.h"

void foo(void);

#if CONFIG_NEOPIXEL && defined(MEGATINYCORE)
#include "Adafruit_seesawPeripheral_tinyneopixel.h"
#endif

#if !defined(CONFIG_EEPROM)
#define CONFIG_EEPROM 1
#endif

#if CONFIG_EEPROM
#include <EEPROM.h>
#define EEPROM_I2C_ADDR (EEPROM.length() - 1)
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

/******** FHT (audio spectrum) */
// FHT is ONLY supported on megaTinyCore (AVR), and will only fit if NO OTHER
// seesaw variants are enabled (i.e. NO GPIO or ADC at same time). Mostly due
// to flash space and/or RAM (literally zero overhead on ATtiny816/817), but
// also because ADC for audio-in requires free-run mode which takes exclusive
// use of the ADC MUX anyway. Input pin is currently #defined here, not
// passed over via Seesaw lib. That might be possible if needed, but since
// there's zero RAM remaining, might have to rely on dirty pool like using
// one of the other ADC registers not in use (TEMP or CTRLE) as a temporary
// holding spot to get that value into Adafruit_seesawPeripheral_reset().
// Also FYI, this depends on the FHT library which is NOT available via the
// Arduino Library Manager and must be separately installed.
#if CONFIG_FHT && defined(MEGATINYCORE)
#if CONFIG_ADC
#error("Cannot enable both CONFIG_ADC and CONFIG_FHT")
#endif
// Currently set up for size 128 FHT (64 spectrum outputs). 256 (128 out)
// is an option IF a larger chip (1K RAM or better) is used; won't fit on
// smaller parts. For now, is set for small part...
#define FHT_N 128
#define LOG_OUT 1
#include <FHT.h>
#define FHT_DEFAULT_PIN 0 // Arduino pin # for input (if no channel select)
#define DISABLE_MILLIS    // FHT is exclusive (no GPIO, etc.), can do this
#endif

uint16_t DATE_CODE = 0;

#define CONFIG_VERSION                                                         \
  (uint32_t)(((uint32_t)PRODUCT_CODE << 16) |                                  \
             ((uint16_t)DATE_CODE & 0x0000FFFF))

/********************** Hardcoded chip configration */

#if defined(ARDUINO_AVR_ATtiny817) || defined(ARDUINO_AVR_ATtiny807) ||        \
    defined(ARDUINO_AVR_ATtiny1617) || defined(ARDUINO_AVR_ATtiny1607) ||      \
    defined(ARDUINO_AVR_ATtiny427) || defined(ARDUINO_AVR_ATtiny827) ||      \
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

#if defined(ARDUINO_AVR_ATtiny817) || defined(ARDUINO_AVR_ATtiny807) ||        \
    defined(ARDUINO_AVR_ATtiny1617) || defined(ARDUINO_AVR_ATtiny1607) ||      \
    defined(ARDUINO_AVR_ATtiny427) || defined(ARDUINO_AVR_ATtiny827) ||      \
    defined(ARDUINO_AVR_ATtiny3217)
#define ALL_GPIO                                                               \
  0x1FFFFFUL // this is chip dependant, for 817 we have 21 GPIO avail (0~20 inc)
#define ALL_ADC 0b1111000000110011001111 // pins that have ADC capability
#ifdef CONFIG_PWM_16BIT
#define ALL_PWM ((1UL << 6) | (1UL << 7) | (1UL << 8))  // alternate TCA0 WOx
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
#define ALL_GPIO                                                               \
  0x01FFFFUL // this is chip dependant, for 816 we have 17 GPIO avail
#define ALL_ADC 0b11100001100111111 // pins that have ADC capability
#ifdef CONFIG_PWM_16BIT
#define ALL_PWM ((1UL << 4) | (1UL << 5) | (1UL << 6))  // alternate TCA0 WOx
#else
#define ALL_PWM                                                                \
  ((1UL << 0) | (1UL << 1) | (1UL << 7) | (1UL << 8) | (1UL << 9) |            \
   (1UL << 10) | (1UL << 11) | (1UL << 16))
#endif
#define PWM_WO_OFFSET (4)
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

#define VALID_GPIO (ALL_GPIO & ~INVALID_GPIO)
#define VALID_ADC (ALL_ADC & VALID_GPIO)
#define VALID_PWM (ALL_PWM & VALID_GPIO)

void Adafruit_seesawPeripheral_reset(void);
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins);
void receiveEvent(int howMany);
void requestEvent(void);
void Adafruit_seesawPeripheral_run(void);
void Adafruit_seesawPeripheral_changedGPIO(void);

/****************************************************** global state */

#if CONFIG_FHT && defined(MEGATINYCORE)
volatile uint8_t i2c_buffer[3]; // Minimal I2C buffer w/FHT because RAM
volatile uint8_t fht_counter;   // For filling FHT input buffer
#else
volatile uint8_t i2c_buffer[32];
#endif

#if CONFIG_INTERRUPT
volatile uint32_t g_irqGPIO = 0;
volatile uint32_t g_irqFlags = 0;
volatile uint8_t IRQ_debounce_cntr = 0;
#define IRQ_DEBOUNCE_TICKS 3 // in millis
#endif

#if CONFIG_ADC
volatile uint8_t g_adcStatus = 0;
#endif
#if (CONFIG_PWM | CONFIG_PWM_16BIT)
volatile uint8_t g_pwmStatus = 0;
#endif
#if CONFIG_NEOPIXEL
volatile uint8_t g_neopixel_buf[CONFIG_NEOPIXEL_BUF_MAX];
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


void Adafruit_seesawPeripheral_setIRQ(void) {
  digitalWrite(CONFIG_INTERRUPT_PIN, LOW);
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT);
}

void Adafruit_seesawPeripheral_clearIRQ(void) {
  // time to turn off the IRQ pin?
  pinMode(CONFIG_INTERRUPT_PIN, INPUT_PULLUP); // open-drainish
}

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

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  return true;
}

void Adafruit_seesawPeripheral_reset(void) {
  Adafruit_seesawPeripheral_setDatecode();

  cli();

  SEESAW_DEBUGLN(F("Wire end"));
  Wire.end();

  // Not referenced after Wire.begin(), so this is now local
  uint8_t _i2c_addr = CONFIG_I2C_PERIPH_ADDR;

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
  for (uint8_t pin = 0; pin < 32; pin++) {
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
  // PWM is provided by BSP's analogWrite() and tone()
#elif CONFIG_PWM_16BIT
  g_pwmStatus = 0;
  // TCA0 is used for 16 bit PWM support
  takeOverTCA0();
  PORTMUX.CTRLC |= 0b111;    // Alternate WOx output pin locations
  TCA0.SINGLE.PER = 0xFFFF;  // Set TOP to MAX
  TCA0.SINGLE.CTRLB = 0x03;  // Single-slope PWM, WG outputs off
  TCA0.SINGLE.CTRLD = 0x00;  // Disable Split Mode
  TCA0.SINGLE.CTRLA = 0x01;  // Enable TCA0 peripheral
#endif
#if CONFIG_NEOPIXEL
  for (uint16_t i = 0; i < CONFIG_NEOPIXEL_BUF_MAX; i++) {
    g_neopixel_buf[i] = 0;
  }
  g_neopixel_bufsize = 0;
#endif
#if CONFIG_ENCODER
#if defined(CONFIG_ENCODER0_A_PIN)
  pinMode(CONFIG_ENCODER0_A_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER0_B_PIN)
  pinMode(CONFIG_ENCODER0_B_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER1_A_PIN)
  pinMode(CONFIG_ENCODER1_A_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER1_B_PIN)
  pinMode(CONFIG_ENCODER1_B_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER2_A_PIN)
  pinMode(CONFIG_ENCODER2_A_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER2_B_PIN)
  pinMode(CONFIG_ENCODER2_B_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER3_A_PIN)
  pinMode(CONFIG_ENCODER3_A_PIN, INPUT_PULLUP);
#endif
#if defined(CONFIG_ENCODER3_B_PIN)
  pinMode(CONFIG_ENCODER3_B_PIN, INPUT_PULLUP);
#endif

  for (uint8_t encodernum=0; encodernum<CONFIG_NUM_ENCODERS; encodernum++) {
    g_enc_value[encodernum] = 0;
    g_enc_delta[encodernum] = 0;
    g_enc_prev_pos[encodernum] = 0;
    g_enc_flags[encodernum] = 0;
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
#ifdef DISABLE_MILLIS
#if defined(MILLIS_USE_TIMERA0)
  TCA0.SPLIT.INTCTRL &= ~TCA_SPLIT_HUNF_bm;
#elif defined(MILLIS_USE_TIMERA1)
  TCA1.SPLIT.INTCTRL &= ~TCA_SPLIT_HUNF_bm;
#elif defined(MILLIS_USE_TIMERB0)
  TCB0.INTCTRL &= ~TCB_CAPT_bm;
#elif defined(MILLIS_USE_TIMERB1)
  TCB1.INTCTRL &= ~TCB_CAPT_bm;
#elif defined(MILLIS_USE_TIMERD0)
  TCD0.INTCTRL &= ~TCD_OVF_bm;
#endif
#endif // end DISABLE_MILLIS

  // ADC is configured for free-run mode with result-ready interrupt. 10-bit
  // w/4X accumulation for 12-bit result (0-4092, NOT 4095, because it's the
  // sum of four 10-bit values, not "true" 12-bit ADC reading).
  // Assuming 10 MHz or 20 MHz F_CPU, possible sampling rates are:
  // 1.25 MHz / 4X samples / 25 ADC cycles/sample -> 12500 Hz sample rate.
  // Highest frequency is 1/2 sampling rate, or 6250 Hz (just under G8 at
  // 6272 Hz). That’s a default that looks nice, but there's some adjustability
  // if needed, with the following top frequency range:
  // 1.25 MHz / 4X / (13+0)  = 24038 sample rate = 12019 peak freq
  // 1.25 MHz / 4X / (13+31) = 7102 sample rate = 3551 peak freq
  // Other F_CPU values (8, 12, 16) will result in different ranges.
  // Depending on what mic is used and its outpot voltage range, might want
  // to change AREF to another source. Right now it's the default VDD.

  fht_counter = 0; // For filling FHT input buffer

  ADC0.CTRLA = ADC_FREERUN_bm | ADC_ENABLE_bm; // 10-bit, free-run, enable ADC
  ADC0.CTRLB = ADC_SAMPNUM_ACC4_gc;   // Accumulate 4X (0-4092 (sic.) result)
  ADC0.CTRLC = ADC_SAMPCAP_bm |       // Reduced capacitance for >1V AREF
               ADC_REFSEL_VDDREF_gc | // VDD as AREF
#if F_CPU > 12000000
               ADC_PRESC_DIV16_gc; // 16:1 timer prescale (20->1.25 MHz)
#else
               ADC_PRESC_DIV8_gc; // 8:1 timer prescale (10->1.25 MHz)
#endif
  ADC0.CTRLD = 0; // No init or sample delay
  ADC0.MUXPOS = digitalPinToAnalogInput(FHT_DEFAULT_PIN);
  ADC0.SAMPCTRL = 12; // Add to usu. 13 ADC cycles for 25 cycles/sample
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt
  ADC0.COMMAND |= ADC_STCONV_bm; // Start free-run conversion
#endif

#if CONFIG_UART
  CONFIG_UART_SERCOM.begin(g_uart_baud);
#endif

  Wire.begin(_i2c_addr);
  sei();
}

#if CONFIG_FHT && defined(MEGATINYCORE)
ISR(ADC0_RESRDY_vect) { // ADC conversion complete
  // Convert 12-bit ADC reading to signed value (+/-2K) and scale to 16-bit
  // space (scaling up isn't strictly required but FHT results look cleaner).
  // 2046 (not 2048) is intentional, see ADC notes above, don't "fix."
  fht_input[fht_counter] = (ADC0.RES - 2046) * 4;
  // Compare-before-increment allows a uint8_t counter, RAM's that tight.
  if (fht_counter == (FHT_N - 1)) { // FHT input buffer full?
    ADC0.INTCTRL &= ~ADC_RESRDY_bm; // Disable result-ready interrupt
  } else {
    fht_counter++;
  }
  // Interrupt flag is cleared automatically when reading ADC0.RES
}
#endif


uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins = VALID_GPIO) {
  uint32_t temp = 0;

  // read all ports
  uint8_t port_reads[3] = {0, 0, 0};
  port_reads[0] = VPORTA.IN;
  port_reads[1] = VPORTB.IN;
  port_reads[2] = VPORTC.IN;

  //pinMode(1, OUTPUT);
  //digitalWriteFast(1, HIGH);
  for (uint8_t pin = 0; pin < 32; pin++) {
    temp >>= 1;
    if (validpins & 0x1) {
      uint8_t mask = 1 << digital_pin_to_bit_position[pin];
      uint8_t port = digital_pin_to_port[pin];
      if (port_reads[port] & mask) {
        temp |= 0x80000000UL;
      }
    }
    validpins >>= 1;
  }
  //digitalWriteFast(1, LOW);
  return temp;
}

void Adafruit_seesawPeripheral_write32(uint32_t value) {
  Wire.write(value >> 24);
  Wire.write(value >> 16);
  Wire.write(value >> 8);
  Wire.write(value);
  return;
}

#include "Adafruit_seesawPeripheral_main.h"
#include "Adafruit_seesawPeripheral_receive.h"
#include "Adafruit_seesawPeripheral_request.h"
#endif
