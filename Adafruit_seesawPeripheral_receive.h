/***************************** data write */

void receiveEvent(int howMany) {
  SEESAW_DEBUG(F("Received "));
  SEESAW_DEBUG(howMany);
  SEESAW_DEBUG(F(" bytes:"));
  // Reject invalid packets before changing the selected module/register.
  if (howMany < 2 || (uint32_t)howMany > sizeof(i2c_buffer)) {
    while (Wire.available()) {
      Wire.read();
    }
    SEESAW_DEBUGLN();
    return;
  }
  for (uint8_t i = 0; i < howMany; i++) {
    i2c_buffer[i] = Wire.read();
    //SEESAW_DEBUG(F("0x"));
    //SEESAW_DEBUG(i2c_buffer[i], HEX);
    //SEESAW_DEBUG(F(" "));
  }
  //SEESAW_DEBUG("\n");

  // Each command validates its length before accessing payload bytes.
  Adafruit_seesawPeripheral_processCommand(i2c_buffer, howMany);
}

/**
 * @brief Decode a command already loaded by a transport.
 * @param packet Command buffer, including module and register.
 * @param howMany Number of received bytes.
 */
void Adafruit_seesawPeripheral_processCommand(const volatile uint8_t *packet,
                                              int howMany) {
  if (howMany < 2) {
    return;
  }
  uint8_t base_cmd = packet[0];
  uint8_t module_cmd = packet[1];

  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_SWRST && howMany == 3 &&
        packet[2] == 0xFF) {
      Adafruit_seesawPeripheral_reset();
      SEESAW_DEBUGLN(F("Resetting"));
    }
  } else if (base_cmd == SEESAW_GPIO_BASE) {
    if (module_cmd == SEESAW_GPIO_BULK && howMany == 2) {
      // Snapshot the pins before the master's following read.
      g_bufferedBulkGPIORead = Adafruit_seesawPeripheral_readBulk(VALID_GPIO);
      return;
    }
    // GPIO writes require the module/register prefix plus a 32-bit value.
    if (howMany != 6) {
      return;
    }
    uint32_t temp = Adafruit_seesawPeripheral_read32(&packet[2]);

    switch (module_cmd) {
      case SEESAW_GPIO_BULK:
        Adafruit_seesawPeripheral_gpioBulk(temp);
        break;
      case SEESAW_GPIO_DIRSET_BULK:
        Adafruit_seesawPeripheral_gpioMode(temp, OUTPUT);
        break;
      case SEESAW_GPIO_DIRCLR_BULK:
        Adafruit_seesawPeripheral_gpioMode(temp, INPUT);
        break;
      case SEESAW_GPIO_BULK_SET:
        Adafruit_seesawPeripheral_gpioWrite(temp, HIGH);
        break;
      case SEESAW_GPIO_BULK_CLR:
        Adafruit_seesawPeripheral_gpioWrite(temp, LOW);
        break;
      case SEESAW_GPIO_PULLENSET:
        Adafruit_seesawPeripheral_gpioMode(temp, INPUT_PULLUP);
        break;
      case SEESAW_GPIO_PULLENCLR:
        Adafruit_seesawPeripheral_gpioMode(temp, INPUT);
        break;
      case SEESAW_GPIO_INTENSET:
      case SEESAW_GPIO_INTENCLR: {
        // Keep one call site to limit code growth on 8 KB targets.
        bool enabled = module_cmd == SEESAW_GPIO_INTENSET;
        Adafruit_seesawPeripheral_gpioInterrupt(temp, enabled);
        break;
      }
    }
  }

#if CONFIG_ADC
  else if (base_cmd == SEESAW_ADC_BASE) {
    // ADC reads contain only the module/register prefix.
    if (howMany != 2) {
      return;
    }
    if (module_cmd >= SEESAW_ADC_CHANNEL_OFFSET) {
      uint8_t adcpin = module_cmd - SEESAW_ADC_CHANNEL_OFFSET;
      if (adcpin >= 32 || !(VALID_ADC & (1UL << adcpin))) {
        g_bufferedADCRead = 0;
        g_adcStatus = 0x1; // error, invalid pin!
      } else {
        // its valid!
        SEESAW_DEBUG(F("ADC read "));
        SEESAW_DEBUG(adcpin);
        SEESAW_DEBUG(F(": "));
        g_bufferedADCRead = Adafruit_seesawPeripheral_readADC(adcpin);
        SEESAW_DEBUGLN(g_bufferedADCRead);
        g_adcStatus = 0x0;
      }
    }
  }
#endif


#if (CONFIG_PWM | CONFIG_PWM_16BIT)
  else if (base_cmd == SEESAW_TIMER_BASE) {
    // Status reads and unknown registers do not contain a PWM payload.
    if (module_cmd != SEESAW_TIMER_PWM && module_cmd != SEESAW_TIMER_FREQ) {
      return;
    }
    // PWM writes contain the prefix, pin, and a 16-bit value.
    if (howMany != 5) {
      g_pwmStatus = 0x1;
      return;
    }
    uint8_t pin = packet[2];
    uint16_t value = Adafruit_seesawPeripheral_read16(&packet[3]);
    if (pin >= 32 || !(VALID_PWM & (1UL << pin))) {
      g_pwmStatus = 0x1; // error, invalid pin!
    } else if (module_cmd == SEESAW_TIMER_PWM) {
      // its valid!
      SEESAW_DEBUG(F("PWM "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);

      Adafruit_seesawPeripheral_setPWM(pin, value);
      g_pwmStatus = 0x0;
    } else if (module_cmd == SEESAW_TIMER_FREQ) {
#if CONFIG_PWM_16BIT && !CONFIG_PWM
      // The 16-bit timer period calculation divides by the frequency.
      if (value == 0) {
        g_pwmStatus = 0x1;
        return;
      }
#endif
      SEESAW_DEBUG(F("Freq "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);
      Adafruit_seesawPeripheral_setPWMFrequency(pin, value);
      g_pwmStatus = 0x0;
    }
  }
#endif

#if CONFIG_NEOPIXEL
  else if (base_cmd == SEESAW_NEOPIXEL_BASE) {
    if (module_cmd == SEESAW_NEOPIXEL_SPEED) {
      // we only support 800khz anyways
    } else if (module_cmd == SEESAW_NEOPIXEL_BUF_LENGTH && howMany == 4) {
      uint16_t value = Adafruit_seesawPeripheral_read16(&packet[2]);
      // dont let it be larger than the internal buffer, of course
      g_neopixel_bufsize = min((uint16_t)CONFIG_NEOPIXEL_BUF_MAX, value);
      SEESAW_DEBUG(F("Neolen "));
      SEESAW_DEBUGLN(g_neopixel_bufsize);
    }
    if (module_cmd == SEESAW_NEOPIXEL_PIN && howMany == 3) {
      uint8_t pin = packet[2];
      if (pin >= 32 || !(VALID_GPIO & (1UL << pin))) {
        return;
      }
      g_neopixel_pin = pin;
      SEESAW_DEBUG(F("Neopin "));
      SEESAW_DEBUGLN(g_neopixel_pin);
    }
    if (module_cmd == SEESAW_NEOPIXEL_BUF && howMany >= 4) {
      uint16_t offset = Adafruit_seesawPeripheral_read16(&packet[2]);
      volatile uint8_t *buffer = Adafruit_seesawPeripheral_pixelBuffer();

      for (uint8_t i = 0; i < howMany - 4; i++) {
        // Widen before adding: a large offset must not wrap into the buffer.
        // Keep accepting the portion of a write that fits the allocated buffer.
        if ((uint32_t)offset + i < CONFIG_NEOPIXEL_BUF_MAX) {
          buffer[offset + i] = packet[4 + i];
        }
      }
    }
    if (module_cmd == SEESAW_NEOPIXEL_SHOW && howMany == 2) {
      Adafruit_seesawPeripheral_showPixels();
    }
  }
#endif

#if CONFIG_ENCODER
  else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t encoder_num = module_cmd & 0x0F;
    uint8_t command = module_cmd & 0xF0;
    if (encoder_num >= CONFIG_NUM_ENCODERS) {
      return;
    }

    if (command == SEESAW_ENCODER_INTENSET && howMany == 3) {
      Adafruit_seesawPeripheral_setEncoderInterrupt(encoder_num, true);
    } else if (command == SEESAW_ENCODER_INTENCLR && howMany == 3) {
      Adafruit_seesawPeripheral_setEncoderInterrupt(encoder_num, false);
    } else if (command == SEESAW_ENCODER_POSITION && howMany == 6) {
      uint32_t value = Adafruit_seesawPeripheral_read32(&packet[2]);
      g_enc_value[encoder_num] = value;
    }
  }
#endif

#if CONFIG_EEPROM
  else if (base_cmd == SEESAW_EEPROM_BASE) {
    // special case for 1 byte at -1 (i2c addr)
    if ((module_cmd == 0xFF) && (howMany == 3)) {
      Adafruit_seesawPeripheral_writeEEPROM(EEPROM_I2C_ADDR, packet[2]);
    } else {
      // write the data
      for (uint8_t i = 0; i < howMany - 2; i++) {
        if ((module_cmd + i) < Adafruit_seesawPeripheral_eepromLength()) {
          Adafruit_seesawPeripheral_writeEEPROM(module_cmd + i, packet[2 + i]);
          SEESAW_DEBUG(F("EEP $"));
          SEESAW_DEBUG(module_cmd+i, HEX);
          SEESAW_DEBUG(F(" <- 0x"));
          SEESAW_DEBUGLN(packet[2+i], HEX);
        }
      }
    }
  }
#endif

#if CONFIG_UART
  else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS) {
      Adafruit_seesawPeripheral_updateUARTStatus();
    } else if ((module_cmd == SEESAW_SERCOM_INTEN) && (howMany == 3)) {
      // writing 0 to this register has no effect
      if (packet[2] & 0x01) {
        g_uart_inten = 1;
      }
    } else if ((module_cmd == SEESAW_SERCOM_INTENCLR) && (howMany == 3)) {
      // writing 0 to this register has no effect
      if (packet[2] & 0x01) {
        g_uart_inten = 0;
      }
    } else if ((module_cmd == SEESAW_SERCOM_BAUD) && (howMany == 6)) {
      uint32_t newBaud = Adafruit_seesawPeripheral_read32(&packet[2]);
      Adafruit_seesawPeripheral_setUARTBaud(newBaud);
    } else if (module_cmd == SEESAW_SERCOM_DATA) {
      Adafruit_seesawPeripheral_queueUARTData(&packet[2], howMany - 2);
    }
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
  else if (base_cmd == SEESAW_SPECTRUM_BASE) {
    if ((module_cmd == SEESAW_SPECTRUM_RATE) && (howMany == 3)) {
      Adafruit_seesawPeripheral_setSpectrumRate(packet[2]);
    } else if ((module_cmd == SEESAW_SPECTRUM_CHANNEL) && (howMany == 3)) {
      ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready IRQ
      uint8_t channel = packet[2];         // Requested ADC channel
      // TO DO: clip channel to valid range. Most likely this will just be
      // 0 or 1 for mic vs. line-in. Value should then be mapped through a
      // const table to either an Arduino pin number (which is then mapped
      // through digitalPinToAnalogInput()) or an ADC MUX value directly
      // (via the datasheet and how the corresponding board gets routed).
      // For now though, for the sake of initial testing, it's taken as a
      // direct ADC MUX value, valid or not. Final changes are only needed
      // here, not in the Adafruit_Seesaw library.
      ADC0.MUXPOS = channel;                   // Set ADC input MUX
      Adafruit_seesawPeripheral_restartSampling(); // Purge recording, start over
    }
  }
#endif
}
