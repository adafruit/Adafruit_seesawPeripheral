/*!
 * @file Adafruit_seesawPeripheral_avr.h
 * @brief AVR hardware access helpers for the shared seesaw protocol.
 */
#ifndef ADAFRUIT_SEESAW_PERIPHERAL_AVR_H
#define ADAFRUIT_SEESAW_PERIPHERAL_AVR_H

#include "wiring_private.h"
#include "pins_arduino.h"

#if CONFIG_NEOPIXEL && defined(MEGATINYCORE)
#include "Adafruit_seesawPeripheral_tinyneopixel.h"
#endif

#if CONFIG_EEPROM
#include <EEPROM.h>

/**
 * @brief Get the hardware EEPROM capacity.
 * @return Capacity in bytes.
 */
uint16_t Adafruit_seesawPeripheral_eepromLength(void) {
  return EEPROM.length();
}

/**
 * @brief Read one byte from hardware EEPROM.
 * @param address EEPROM byte address.
 * @return Stored byte.
 */
uint8_t Adafruit_seesawPeripheral_readEEPROM(uint16_t address) {
  return EEPROM.read(address);
}

/**
 * @brief Write one byte to hardware EEPROM.
 * @param address EEPROM byte address.
 * @param value Byte to store.
 */
void Adafruit_seesawPeripheral_writeEEPROM(uint16_t address, uint8_t value) {
  EEPROM.write(address, value);
}
#endif

#if CONFIG_ADC
/**
 * @brief Read an ADC pin already checked by the command decoder.
 * @param pin Arduino pin number.
 * @return Raw ADC reading.
 */
uint16_t Adafruit_seesawPeripheral_readADC(uint8_t pin) {
  return analogRead(pin);
}
#endif

#if CONFIG_PWM || CONFIG_PWM_16BIT
/**
 * @brief Set the hardware PWM duty cycle.
 * @param pin PWM output pin.
 * @param value 16-bit duty setting, from 0 through 65535.
 */
void Adafruit_seesawPeripheral_setPWM(uint8_t pin, uint16_t value) {
  pinMode(pin, OUTPUT);
#if CONFIG_PWM
  analogWrite(pin, value >> 8); // 8-bit PWM uses the high byte.
#elif CONFIG_PWM_16BIT
  // set duty cycle via CMPx
  uint16_t duty_cycle = map(value, 0, 0xFFFF, 0, TCA0.SINGLE.PER);
  pin -= PWM_WO_OFFSET;
  if (pin == 0) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;
    TCA0.SINGLE.CMP2 = duty_cycle;
  } else if (pin == 1) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP1EN_bm;
    TCA0.SINGLE.CMP1 = duty_cycle;
  } else if (pin == 2) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CMP0 = duty_cycle;
  }
#endif
}

/**
 * @brief Set the hardware PWM frequency.
 * @param pin PWM output pin.
 * @param frequency Frequency in hertz; must be nonzero for 16-bit PWM.
 */
void Adafruit_seesawPeripheral_setPWMFrequency(uint8_t pin,
                                               uint16_t frequency) {
#if CONFIG_PWM
  tone(pin, frequency);
#elif CONFIG_PWM_16BIT
  pinMode(pin, OUTPUT);
  // set frequency via CLKSEL and PER based on F_CPU
  // NOTE: changes all PWM outputs
  uint8_t clksel = 0;
  unsigned long period = (F_CPU / frequency);
  while (period > 65536 && clksel < 7) {
    clksel++;
    period = period >> (clksel > 4 ? 2 : 1);
  }
  TCA0.SINGLE.CTRLA = (clksel << 1) | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.PER = period;
#endif
}
#endif

#if CONFIG_NEOPIXEL
/**
 * @brief Get the fixed NeoPixel buffer.
 * @return Writable pixel buffer.
 */
volatile uint8_t *Adafruit_seesawPeripheral_pixelBuffer(void) {
  return g_neopixel_buf;
}

/**
 * @brief Send the configured NeoPixel buffer to the selected output pin.
 */
void Adafruit_seesawPeripheral_showPixels(void) {
  pinMode(g_neopixel_pin, OUTPUT);
  tinyNeoPixel_show(g_neopixel_pin, g_neopixel_bufsize,
                    (uint8_t *)g_neopixel_buf);
}
#endif

#if CONFIG_UART
/**
 * @brief Refresh the UART data-ready bit, preserving other status flags.
 */
void Adafruit_seesawPeripheral_updateUARTStatus(void) {
  if (CONFIG_UART_SERCOM.available()) {
    g_uart_status |= 0x02; // set DATA_RDY bit
  } else {
    g_uart_status &= ~0x02; // clear DATA_RDY bit
  }
}

/**
 * @brief Change the UART baud rate only when needed.
 * @param baud Baud rate in bits per second.
 */
void Adafruit_seesawPeripheral_setUARTBaud(uint32_t baud) {
  if (baud != g_uart_baud) {
    g_uart_baud = baud;
    CONFIG_UART_SERCOM.end();
    CONFIG_UART_SERCOM.begin(g_uart_baud);
  }
}

/**
 * @brief Read one byte from the UART receive buffer.
 * @return Byte value, or -1 if no data is available.
 */
int Adafruit_seesawPeripheral_readUARTData(void) {
  return CONFIG_UART_SERCOM.read();
}
#endif

/**
 * @brief Apply address straps to the base I2C address.
 * @param address Base I2C address.
 * @return Address after applying configured straps.
 */
uint8_t Adafruit_seesawPeripheral_applyAddressStraps(uint8_t address) {
#if CONFIG_ADDR_0
  pinMode(CONFIG_ADDR_0_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_0_PIN) == CONFIG_ADDR_INVERTED)
    address += 1;
#endif
#if CONFIG_ADDR_1
  pinMode(CONFIG_ADDR_1_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_1_PIN) == CONFIG_ADDR_INVERTED)
    address += 2;
#endif
#if CONFIG_ADDR_2
  pinMode(CONFIG_ADDR_2_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_2_PIN) == CONFIG_ADDR_INVERTED)
    address += 4;
#endif
#if CONFIG_ADDR_3
  pinMode(CONFIG_ADDR_3_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_3_PIN) == CONFIG_ADDR_INVERTED)
    address += 8;
#endif
  return address;
}

void Adafruit_seesawPeripheral_setIRQ(void) {
  digitalWrite(CONFIG_INTERRUPT_PIN, LOW);
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT);
}

void Adafruit_seesawPeripheral_clearIRQ(void) {
  // time to turn off the IRQ pin?
  pinMode(CONFIG_INTERRUPT_PIN, INPUT_PULLUP); // open-drainish
}

#if CONFIG_INTERRUPT
/**
 * @brief Enable or disable change interrupts for one GPIO pin.
 * @param pin GPIO pin to configure.
 * @param enabled True to enable interrupts, false to disable them.
 */
void Adafruit_seesawPeripheral_setGPIOInterrupt(uint8_t pin, bool enabled) {
  if (enabled) {
    g_irqGPIO |= 1UL << pin;
    SEESAW_DEBUGLN(F(" INTEN"));
#if USE_PINCHANGE_INTERRUPT
    attachInterrupt(digitalPinToInterrupt(pin),
                    Adafruit_seesawPeripheral_changedGPIO, CHANGE);
#endif
  } else {
    g_irqGPIO &= ~(1UL << pin);
    SEESAW_DEBUGLN(F(" INTCLR"));
#if USE_PINCHANGE_INTERRUPT
    detachInterrupt(digitalPinToInterrupt(pin));
#endif
  }
}
#endif

/**
 * @brief Set the pin mode of selected valid GPIO pins.
 * @param mask GPIO bits to change; invalid pins are ignored.
 * @param mode Arduino pin mode, such as INPUT, OUTPUT, or INPUT_PULLUP.
 */
void Adafruit_seesawPeripheral_gpioMode(uint32_t mask, uint8_t mode) {
  mask &= VALID_GPIO;
  for (uint8_t pin = 0; pin < 32; pin++) {
    if ((mask >> pin) & 0x1) {
      SEESAW_DEBUG(F("Set pin "));
      SEESAW_DEBUG(pin);
      pinMode(pin, mode);
      SEESAW_DEBUG(F(" mode "));
      SEESAW_DEBUGLN(mode);
    }
  }
}

/**
 * @brief Write a level to selected valid GPIO pins.
 * @param mask GPIO bits to change; invalid pins are ignored.
 * @param value Arduino pin level, HIGH or LOW.
 */
void Adafruit_seesawPeripheral_gpioWrite(uint32_t mask, uint8_t value) {
  mask &= VALID_GPIO;
  for (uint8_t pin = 0; pin < 32; pin++) {
    if ((mask >> pin) & 0x1) {
      SEESAW_DEBUG(F("Set pin "));
      SEESAW_DEBUG(pin);
      digitalWrite(pin, value);
      SEESAW_DEBUG(F(" value "));
      SEESAW_DEBUGLN(value);
    }
  }
}

/**
 * @brief Write all valid GPIO pins from a bulk value.
 * @param value Pin levels, with bit N controlling GPIO N.
 */
void Adafruit_seesawPeripheral_gpioBulk(uint32_t value) {
  uint32_t pinmask = 0x1;
  for (uint8_t pin = 0; pin < 32; pin++) {
    if (VALID_GPIO & pinmask) {
      digitalWrite(pin, (value >> pin) & 0x1);
    }
    pinmask <<= 1;
  }
}

uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins = VALID_GPIO) {
  uint32_t temp = 0;

  // read all ports
  uint8_t port_reads[3] = {0, 0, 0};
  port_reads[0] = VPORTA.IN;
  port_reads[1] = VPORTB.IN;
  port_reads[2] = VPORTC.IN;

  // pinMode(1, OUTPUT);
  // digitalWriteFast(1, HIGH);
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
  // digitalWriteFast(1, LOW);
  return temp;
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

/**
 * @brief Restart FHT capture after an ADC sampling rate or MUX channel change.
 * Discard three initial readings, then reset the FHT buffer counter.
 */
static void Adafruit_seesawPeripheral_restartSampling(void) {
  for (uint8_t i = 0; i < 3; i++) { // Discard initial readings
    while (!ADC0.INTFLAGS & ADC_RESRDY_bm)
      ;                             // Wait for a conversion.
    ADC0.INTFLAGS |= ADC_RESRDY_bm; // Setting the bit clears the flag.
    // ADC is still free-running and will set RESRDY;
    // only its interrupts are disabled right now.
  }
  fht_counter = 0;               // Restart at beginning of buffer.
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready IRQ.
}

/**
 * @brief Set the FHT sample-rate index and restart capture.
 * @param rate Sample-rate index, clamped to 0-31.
 */
void Adafruit_seesawPeripheral_setSpectrumRate(uint8_t rate) {
  ADC0.INTCTRL &= ~ADC_RESRDY_bm; // Disable result-ready IRQ.
  if (rate > 31) {
    rate = 31;
  }
  ADC0.SAMPCTRL = rate & 31;
  Adafruit_seesawPeripheral_restartSampling();
}

/**
 * @brief Read the current FHT sample-rate index.
 * @return Raw ADC sample-control value.
 */
uint8_t Adafruit_seesawPeripheral_readSpectrumRate(void) {
  return ADC0.SAMPCTRL;
}

/**
 * @brief Select the FHT ADC input and restart capture.
 * @param channel Raw ADC MUX value, passed through without validation.
 */
void Adafruit_seesawPeripheral_setSpectrumChannel(uint8_t channel) {
  ADC0.INTCTRL &= ~ADC_RESRDY_bm; // Disable result-ready IRQ.
  // TO DO: clip channel to valid range. Most likely this will just be
  // 0 or 1 for mic vs. line-in. Value should then be mapped through a
  // const table to either an Arduino pin number (which is then mapped
  // through digitalPinToAnalogInput()) or an ADC MUX value directly
  // (via the datasheet and how the corresponding board gets routed).
  // For now, preserve the raw ADC MUX value, valid or not. Any future
  // channel mapping must also be used by readSpectrumChannel().
  ADC0.MUXPOS = channel;
  Adafruit_seesawPeripheral_restartSampling();
}

/**
 * @brief Read the FHT ADC input selection without channel remapping.
 * @return Raw ADC MUX value.
 */
uint8_t Adafruit_seesawPeripheral_readSpectrumChannel(void) {
  return ADC0.MUXPOS;
}

/**
 * @brief Get the current logarithmic FHT output buffer.
 * @return Read-only pointer to the FHT_N / 2 result bytes.
 */
const uint8_t *Adafruit_seesawPeripheral_spectrumBuffer(void) {
  return fht_log_out;
}
#endif

#endif
