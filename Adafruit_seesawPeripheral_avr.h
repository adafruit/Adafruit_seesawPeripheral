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
#include "fht_tables.h"

volatile uint8_t fht_counter; // For filling FHT input buffer

/**
 * @brief Configure free-running ADC capture with the existing FHT defaults.
 */
void Adafruit_seesawPeripheral_initializeSpectrum(void) {
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
}

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

/**
 * @brief Process one captured FHT frame and resume sampling.
 */
void Adafruit_seesawPeripheral_runSpectrum(void) {
  while (ADC0.INTCTRL & ADC_RESRDY_bm)
    ; // Wait for sampling to finish

  fht_window();
  fht_reorder();
  fht_run();

  // I2C interrupts are temporarily disabled so we don't return spectrum
  // data in the middle of it being processed. If an I2C event occurs,
  // the corresponding interrupt flag will still get set, it just won't
  // act on it until later when the interrupts are re-enabled.
  // Up to this point though, everything's happening in fht_input[].
  Adafruit_seesawPeripheral_enableI2CInterrupts(false);

  // FHT has various output modes, after much testing settled on this...
  fht_mag_log(); // -> fht_log_out[], if using this, define LOG_OUT to 1

  // Resume sampling at earliest opportunity...
  fht_counter = 0;
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt

  // Process and/or dump FHT output
  for (uint8_t i = 0; i < FHT_N / 2; i++) {
    if (fht_log_out[i] > noise[i]) {
      fht_log_out[i] -= noise[i];
      if (fht_log_out[i] < peak[i]) {
        // Scale column to make up for range lost to noise subtraction.
        // Initially thought some additional per-column scaling would be
        // needed to boost the highs, but when testing with pure tones
        // the response across the graph looks reasonably linear-ish
        // enough, the lower peaks at the top are really just a function
        // of how music is. Host-side code can do some auto-scale
        // massaging of the data if really desired (a little frame-to-
        // frame filtering there looks nice anyway).
        fht_log_out[i] = (fht_log_out[i] * scale[i]) >> 8;
      } else {
        fht_log_out[i] = 255; // At or above peak
      }
    } else {
      fht_log_out[i] = 0; // At or below noise threshold
    }
    // Note to future self: noise and peak are a function of the
    // microphone, and they do seem to vary with sampling rate.
    // There's ample flash remaining, one possibility is to use
    // different tables depending on the active rate setting.
    // Would want to record those on final hardware.
  }

  // Re-enable I2C interrupts -- safe to issue spectrum data
  Adafruit_seesawPeripheral_enableI2CInterrupts(true);
}
#endif

/**
 * @brief Enable or disable global interrupts during peripheral reset.
 * @param enabled True to enable interrupts, false to disable them.
 */
__attribute__((always_inline)) inline void
Adafruit_seesawPeripheral_enableInterrupts(bool enabled) {
  if (enabled) {
    sei();
  } else {
    cli();
  }
}

/**
 * @brief Reset valid GPIO pins and detach configured change interrupts.
 */
void Adafruit_seesawPeripheral_initializeGPIO(void) {
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
}

#if CONFIG_PWM || CONFIG_PWM_16BIT
/**
 * @brief Reset PWM status and configure the selected hardware PWM mode.
 */
void Adafruit_seesawPeripheral_initializePWM(void) {
#if CONFIG_PWM
  g_pwmStatus = 0;
  // PWM is provided by BSP's analogWrite() and tone()
#elif CONFIG_PWM_16BIT
  g_pwmStatus = 0;
  // TCA0 is used for 16 bit PWM support
  takeOverTCA0();
  PORTMUX.CTRLC |= 0b111;   // Alternate WOx output pin locations
  TCA0.SINGLE.PER = 0xFFFF; // Set TOP to MAX
  TCA0.SINGLE.CTRLB = 0x03; // Single-slope PWM, WG outputs off
  TCA0.SINGLE.CTRLD = 0x00; // Disable Split Mode
  TCA0.SINGLE.CTRLA = 0x01; // Enable TCA0 peripheral
#endif
}
#endif

#if CONFIG_ENCODER
/**
 * @brief Enable pull-ups on configured encoder input pins.
 */
void Adafruit_seesawPeripheral_initializeEncoderPins(void) {
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
}
#endif

#if CONFIG_UART
/**
 * @brief Start the UART with the current baud setting.
 */
void Adafruit_seesawPeripheral_initializeUART(void) {
  CONFIG_UART_SERCOM.begin(g_uart_baud);
}

/**
 * @brief Update the UART interrupt output and drain queued transmit bytes.
 */
void Adafruit_seesawPeripheral_runUART(void) {
  if (g_uart_inten) {
    if (CONFIG_UART_SERCOM.available()) {
      Adafruit_seesawPeripheral_setIRQ();
    } else {
      Adafruit_seesawPeripheral_clearIRQ();
    }
  }
  if (g_uart_tx_len > 0) {
    CONFIG_UART_SERCOM.write((char *)g_uart_buf, (size_t)g_uart_tx_len);
    g_uart_tx_len = 0;
  }
}
#endif

/**
 * @brief Set GPIO direction using the established AVR pin-mode behavior.
 * @param mask GPIO bits to change.
 * @param enabled True for output, false for input.
 */
void Adafruit_seesawPeripheral_gpioDirection(uint32_t mask, bool enabled) {
  if (enabled) {
    Adafruit_seesawPeripheral_gpioMode(mask, OUTPUT);
  } else {
    Adafruit_seesawPeripheral_gpioMode(mask, INPUT);
  }
}

/**
 * @brief Set GPIO pulls using the established AVR pin-mode behavior.
 * @param mask GPIO bits to change.
 * @param enabled True for pull-up input, false for plain input.
 */
void Adafruit_seesawPeripheral_gpioPull(uint32_t mask, bool enabled) {
  if (enabled) {
    Adafruit_seesawPeripheral_gpioMode(mask, INPUT_PULLUP);
  } else {
    Adafruit_seesawPeripheral_gpioMode(mask, INPUT);
  }
}

#endif
