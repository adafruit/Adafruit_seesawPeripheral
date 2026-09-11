/*!
 * @file Adafruit_seesawPeripheral_receive.h
 * Shared seesaw command decoding. AVR dispatches in its Wire callback; STM32
 * passes queued packets here from loop(), keeping blocking work out of IRQs.
 */

#if CONFIG_FHT && defined(MEGATINYCORE)
// If ADC sampling rate or MUX channel is changed, this function gets
// called to discard a couple of initial ADC readings (which are invalid
// immdiately after such a change) and reset the FHT buffer counter to
// the beginning.
static void restart_sampling(void) {
  for(uint8_t i=0; i<3; i++) {             // Discard initial readings
    while(!ADC0.INTFLAGS & ADC_RESRDY_bm); // In the INTFLAG register,
    ADC0.INTFLAGS |= ADC_RESRDY_bm;        // setting bit clears flag!
    // (ADC is still free-running and will set RESRDY bit,
    // it's just not triggering interrupts right now.)
  }
  fht_counter = 0;                         // Restart at beginning of buf
  ADC0.INTCTRL |= ADC_RESRDY_bm;           // Enable result-ready IRQ
}
#endif

/*! Decode one complete packet, including its module/register prefix. */
void Adafruit_seesawPeripheral_processCommand(const uint8_t *packet,
                                              uint8_t howMany) {
  if (howMany < 2)
    return;
  uint8_t base_cmd = packet[0];
  uint8_t module_cmd = packet[1];
  const uint8_t *data = packet + 2;
  uint8_t length = howMany - 2;
#if defined(ARDUINO_ARCH_STM32)
  using namespace SeesawSTM32;
#endif
#if CONFIG_SPI && defined(ARDUINO_ARCH_STM32)
  if (base_cmd == spiBase) {
    spiProcess(module_cmd, data, length);
    noInterrupts();
    if (spiPending)
      spiPending--;
    interrupts();
    return;
  }
#endif

  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_SWRST && length == 1 && data[0] == 0xFF)
      Adafruit_seesawPeripheral_reset();
  } else if (base_cmd == SEESAW_GPIO_BASE) {
    if (module_cmd == SEESAW_GPIO_BULK && length == 0) {
#if !defined(ARDUINO_ARCH_STM32)
      // The AVR port snapshots before the master's following read.
      g_bufferedBulkGPIORead = Adafruit_seesawPeripheral_readBulk(VALID_GPIO);
#endif
      return;
    }
    if (length != 4)
      return;
    uint32_t temp = Adafruit_seesawPeripheral_read32(data) & VALID_GPIO;
    switch (module_cmd) {
    case SEESAW_GPIO_DIRSET_BULK:
      Adafruit_seesawPeripheral_gpioDirection(temp, true);
      break;
    case SEESAW_GPIO_DIRCLR_BULK:
      Adafruit_seesawPeripheral_gpioDirection(temp, false);
      break;
    case SEESAW_GPIO_BULK:
      Adafruit_seesawPeripheral_gpioWrite(VALID_GPIO & ~temp, false);
      Adafruit_seesawPeripheral_gpioWrite(temp, true);
      break;
    case SEESAW_GPIO_BULK_SET:
      Adafruit_seesawPeripheral_gpioWrite(temp, true);
      break;
    case SEESAW_GPIO_BULK_CLR:
      Adafruit_seesawPeripheral_gpioWrite(temp, false);
      break;
    case SEESAW_GPIO_BULK_TOGGLE:
      Adafruit_seesawPeripheral_gpioToggle(temp);
      break;
    case SEESAW_GPIO_PULLENSET:
      Adafruit_seesawPeripheral_gpioPull(temp, true);
      break;
    case SEESAW_GPIO_PULLENCLR:
      Adafruit_seesawPeripheral_gpioPull(temp, false);
      break;
    case SEESAW_GPIO_INTENSET:
      Adafruit_seesawPeripheral_gpioInterrupt(temp, true);
      break;
    case SEESAW_GPIO_INTENCLR:
      Adafruit_seesawPeripheral_gpioInterrupt(temp, false);
      break;
    }
  }
#if CONFIG_ADC
  else if (base_cmd == SEESAW_ADC_BASE && length == 0 &&
             module_cmd >= SEESAW_ADC_CHANNEL_OFFSET) {
    uint8_t adcpin = module_cmd - SEESAW_ADC_CHANNEL_OFFSET;
    g_adcStatus = 1;
    g_bufferedADCRead = 0;
    if (adcpin < 32 && (VALID_ADC & (1UL << adcpin))) {
#if defined(ARDUINO_ARCH_STM32)
      uint16_t value;
      if (!readADC(adcpin, value))
        return;
      g_bufferedADCRead = value;
#else
      g_bufferedADCRead = analogRead(adcpin);
#endif
      g_adcStatus = 0;
    }
  }
#endif

#if CONFIG_PWM || CONFIG_PWM_16BIT
  else if (base_cmd == SEESAW_TIMER_BASE && length == 3) {
    uint8_t pin = data[0];
    uint16_t value = Adafruit_seesawPeripheral_read16(data + 1);
    g_pwmStatus = 1;
    if (pin >= 32 || ! (VALID_PWM & (1UL << pin)))
      return;
    if (module_cmd == SEESAW_TIMER_PWM)
      Adafruit_seesawPeripheral_setPWM(pin, value);
    else if (module_cmd == SEESAW_TIMER_FREQ && value != 0)
      Adafruit_seesawPeripheral_setPWMFrequency(pin, value);
    else
      return;
    g_pwmStatus = 0;
  }
#endif

#if CONFIG_NEOPIXEL
  else if (base_cmd == SEESAW_NEOPIXEL_BASE) {
    g_neopixel_status = 0;
    if (module_cmd == SEESAW_NEOPIXEL_PIN && length == 1) {
      if (data[0] >= 32 || !(VALID_GPIO & (1UL << data[0]))) {
        g_neopixel_status = 1;
        return;
      }
      g_neopixel_pin = data[0];
      Adafruit_seesawPeripheral_setPixelPin(g_neopixel_pin);
    } else if (module_cmd == SEESAW_NEOPIXEL_SPEED && length == 1) {
      // Both existing hardware implementations support 800 kHz only.
      if (data[0] != 1)
        g_neopixel_status = 1;
    } else if (module_cmd == SEESAW_NEOPIXEL_BUF_LENGTH && length == 2) {
      uint16_t value = Adafruit_seesawPeripheral_read16(data);
      g_neopixel_bufsize = min(value, (uint16_t)CONFIG_NEOPIXEL_BUF_MAX);
      Adafruit_seesawPeripheral_setPixelLength(g_neopixel_bufsize);
    } else if (module_cmd == SEESAW_NEOPIXEL_BUF && length >= 2) {
      uint16_t offset = Adafruit_seesawPeripheral_read16(data);
      volatile uint8_t *buffer = Adafruit_seesawPeripheral_pixelBuffer();
      if (!buffer || (uint32_t)offset + length - 2 > CONFIG_NEOPIXEL_BUF_MAX) {
        g_neopixel_status = 1;
        return;
      }
      for (uint8_t i = 2; i < length; i++)
        buffer[offset + i - 2] = data[i];
    } else if (module_cmd == SEESAW_NEOPIXEL_SHOW && length == 0) {
      if (g_neopixel_pin >= 32 || !g_neopixel_bufsize ||
          !Adafruit_seesawPeripheral_pixelBuffer()) {
        g_neopixel_status = 1;
        return;
      }
      Adafruit_seesawPeripheral_showPixels();
    }
  }
#endif

#if CONFIG_ENCODER
  else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t encoder_num = module_cmd & 0x0F;
    if (encoder_num >= CONFIG_NUM_ENCODERS)
      return;
    switch (module_cmd & 0xF0) {
    case SEESAW_ENCODER_INTENSET:
    case SEESAW_ENCODER_INTENCLR: {
      bool enabled = (module_cmd & 0xF0) == SEESAW_ENCODER_INTENSET;
#if defined(ARDUINO_ARCH_STM32)
      if (enabled)
        encoderInterrupts |= 1 << encoder_num;
      else
        encoderInterrupts &= ~(1 << encoder_num);
#else
      // AVR's edge detector owns GPIO IRQ masks; STM32 latches detent IRQs.
      uint32_t mask = 0;
      if (encoder_num == 0)
        mask = ENCODER0_INPUT_MASK;
      if (encoder_num == 1)
        mask = ENCODER1_INPUT_MASK;
      if (encoder_num == 2)
        mask = ENCODER2_INPUT_MASK;
      if (encoder_num == 3)
        mask = ENCODER3_INPUT_MASK;
      if (enabled)
        g_irqGPIO |= mask;
      else
        g_irqGPIO &= ~mask;
#endif
      break;
    }
    case SEESAW_ENCODER_POSITION:
      if (length == 4) {
#if defined(ARDUINO_ARCH_STM32)
        noInterrupts(); // AVR already executes inside the Wire callback.
#endif
        g_enc_value[encoder_num] = (int32_t)Adafruit_seesawPeripheral_read32(data);
#if defined(ARDUINO_ARCH_STM32)
        g_enc_delta[encoder_num] = 0;
        interrupts();
#endif
      }
      break;
    }
  }
#endif

#if CONFIG_EEPROM
  else if (base_cmd == SEESAW_EEPROM_BASE && length > 0) {
#if defined(ARDUINO_ARCH_STM32)
    // Flash erases/programs a reserved page; AVR has byte-addressable EEPROM.
    writeEEPROM(module_cmd, data, length);
#else
    if (module_cmd == 0xFF && length == 1)
      EEPROM.write(EEPROM.length()-1, data[0]);
    else
      for (uint8_t i=0; i< length; i++)
        if ((uint16_t)module_cmd + i < EEPROM.length())
          EEPROM.write(module_cmd + i, data[i]);
#endif
  }
#endif

#if CONFIG_UART
  else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS && length == 0)
      g_uart_status = CONFIG_UART_SERCOM.available() ? 2 : 0;
    else if (module_cmd == SEESAW_SERCOM_INTEN && length == 1)
      g_uart_inten |= data[0] & 1;
    else if (module_cmd == SEESAW_SERCOM_INTENCLR && length == 1)
      g_uart_inten &= ~(data[0] & 1);
    else if (module_cmd == SEESAW_SERCOM_BAUD && length == 4) {
      uint32_t newBaud = Adafruit_seesawPeripheral_read32(data);
#if defined(ARDUINO_ARCH_STM32)
      if (newBaud < 300 || newBaud > 1000000)
        return;
#endif
      if (newBaud != g_uart_baud) {
        g_uart_baud = newBaud;
        CONFIG_UART_SERCOM.end();
        CONFIG_UART_SERCOM.begin(newBaud);
      }
    } else if (module_cmd == SEESAW_SERCOM_DATA && length > 0) {
#if defined(ARDUINO_ARCH_STM32)
      CONFIG_UART_SERCOM.write(data, length); // Already in the deferred queue.
#else
      // AVR dispatches in IRQ context, so the main loop drains UART data.
      g_uart_tx_len = 0;
      if (length <= CONFIG_UART_BUF_MAX) {
        for (uint8_t i=0; i< length; i++)
          g_uart_buf[i] = data[i];
        g_uart_tx_len = length;
      }
#endif
    }
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
  else if (base_cmd == SEESAW_SPECTRUM_BASE) {
    if ((module_cmd == SEESAW_SPECTRUM_RATE) && (howMany == 3)) {
      ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready IRQ
      uint8_t rate = data[0];            // Requested rate index
      if (rate > 31) rate = 31;                // Clip rate between 0-31
      ADC0.SAMPCTRL = rate & 31;               // Set ADC sample control
      restart_sampling();                      // Purge recording, start over
    } else if ((module_cmd == SEESAW_SPECTRUM_CHANNEL) && (howMany == 3)) {
      ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready IRQ
      uint8_t channel = data[0];         // Requested ADC channel
      // TO DO: clip channel to valid range. Most likely this will just be
      // 0 or 1 for mic vs. line-in. Value should then be mapped through a
      // const table to either an Arduino pin number (which is then mapped
      // through digitalPinToAnalogInput()) or an ADC MUX value directly
      // (via the datasheet and how the corresponding board gets routed).
      // For now though, for the sake of initial testing, it's taken as a
      // direct ADC MUX value, valid or not. Final changes are only needed
      // here, not in the Adafruit_Seesaw library.
      ADC0.MUXPOS = channel;                   // Set ADC input MUX
      restart_sampling();                      // Purge recording, start over
    }
  }
#endif
}

/*! Wire receive entry point. Only STM32 needs the deferred command queue. */
void receiveEvent(int howMany) {
#if defined(ARDUINO_ARCH_STM32)
  SeesawSTM32::receive(howMany);
#else
  if (howMany < 2 || (uint32_t)howMany > sizeof(i2c_buffer)) {
    while (Wire.available())
      Wire.read();
    return;
  }
  for (uint8_t i = 0; i < howMany; i++)
    i2c_buffer[i] = Wire.read();
  // This callback owns the packet throughout decoding; it cannot be replaced
  // by a second receive callback while interrupts are disabled.
  Adafruit_seesawPeripheral_processCommand((const uint8_t *)i2c_buffer,
                                           howMany);
#endif
}
