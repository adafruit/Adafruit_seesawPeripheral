/**
 * @brief Write a 16-bit value to the I2C reply, most significant byte
 * first.
 * @param value Value to send.
 */
__attribute__((always_inline)) inline void
Adafruit_seesawPeripheral_write16(uint16_t value) {
  Adafruit_seesawPeripheral_write8((uint8_t)(value >> 8));
  Adafruit_seesawPeripheral_write8((uint8_t)value);
}

/**
 * @brief Write a 32-bit I2C reply, most significant byte first.
 * @param value Value to send.
 */
__attribute__((always_inline)) inline void
Adafruit_seesawPeripheral_write32(uint32_t value) {
  Adafruit_seesawPeripheral_write8((uint8_t)(value >> 24));
  Adafruit_seesawPeripheral_write8((uint8_t)(value >> 16));
  Adafruit_seesawPeripheral_write8((uint8_t)(value >> 8));
  Adafruit_seesawPeripheral_write8((uint8_t)value);
}

/**
 * @brief Encode a register reply using the transport's reply helpers.
 * @param base_cmd Module selected by the transport.
 * @param module_cmd Register selected by the transport.
 */
void Adafruit_seesawPeripheral_processRequest(uint8_t base_cmd,
                                              uint8_t module_cmd) {
#if CONFIG_SPI
  if (base_cmd == SeesawSTM32::spiBase) {
    SeesawSTM32::spiRequest(module_cmd);
    return;
  }
#endif
  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_HW_ID) {
      Adafruit_seesawPeripheral_write8(SEESAW_HW_ID); // instant reply
    }
    if (module_cmd == SEESAW_STATUS_VERSION) {
      Adafruit_seesawPeripheral_write32(CONFIG_VERSION); // instant reply
    }
#if defined(ARDUINO_ARCH_STM32) && CONFIG_ADC
    else if (module_cmd == SEESAW_STATUS_TEMP) {
      Adafruit_seesawPeripheral_write32(SeesawSTM32::temperature);
    }
#endif
    else if (module_cmd == SEESAW_STATUS_OPTIONS) {
      uint32_t options =
          (1UL << SEESAW_STATUS_BASE) | (1UL << SEESAW_GPIO_BASE) |
          ((uint32_t)CONFIG_ADC << SEESAW_ADC_BASE) |
          ((uint32_t)(CONFIG_PWM || CONFIG_PWM_16BIT) << SEESAW_TIMER_BASE) |
          ((uint32_t)CONFIG_EEPROM << SEESAW_EEPROM_BASE) |
          ((uint32_t)CONFIG_NEOPIXEL << SEESAW_NEOPIXEL_BASE) |
          ((uint32_t)CONFIG_UART << SEESAW_SERCOM0_BASE) |
          ((uint32_t)CONFIG_ENCODER << SEESAW_ENCODER_BASE) |
          ((uint32_t)CONFIG_FHT << SEESAW_SPECTRUM_BASE);
#if CONFIG_SPI
      options |= 1UL << SeesawSTM32::spiBase;
#endif
      Adafruit_seesawPeripheral_write32(options);
    }
  } else if (base_cmd == SEESAW_GPIO_BASE) {
    if (module_cmd == SEESAW_GPIO_BULK) {
      Adafruit_seesawPeripheral_write32(g_bufferedBulkGPIORead); // instant reply because we did the write before
#if CONFIG_INTERRUPT
      g_irqFlags = 0; // reading the gpio pins clears them
      Adafruit_seesawPeripheral_clearIRQ();
#endif
    }
#if CONFIG_INTERRUPT
    else if (module_cmd == SEESAW_GPIO_INTFLAG) {
      Adafruit_seesawPeripheral_write32(g_irqFlags);
      g_irqFlags = 0; // reading the flags clears them
      Adafruit_seesawPeripheral_clearIRQ();
    }
#endif
  }

#if CONFIG_ADC
  else if (base_cmd == SEESAW_ADC_BASE) {
    if (module_cmd >= SEESAW_ADC_CHANNEL_OFFSET) {
      Adafruit_seesawPeripheral_write16(g_bufferedADCRead);
    } else if (module_cmd == SEESAW_ADC_STATUS) {
      Adafruit_seesawPeripheral_write8(g_adcStatus);
    }
  }
#endif

#if CONFIG_PWM || CONFIG_PWM_16BIT
  else if (base_cmd == SEESAW_TIMER_BASE) {
    if (module_cmd == SEESAW_TIMER_STATUS) {
      Adafruit_seesawPeripheral_write8(g_pwmStatus);
    }
  }
#endif

#if CONFIG_EEPROM
  else if (base_cmd == SEESAW_EEPROM_BASE) {
    Adafruit_seesawPeripheral_write8(Adafruit_seesawPeripheral_readEEPROM(
        module_cmd % Adafruit_seesawPeripheral_eepromLength()));
  }
#endif

#if CONFIG_ENCODER
  else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t encoder_num = module_cmd & 0x0F;
    uint8_t command = module_cmd & 0xF0;
    if (encoder_num < CONFIG_NUM_ENCODERS &&
        (command == SEESAW_ENCODER_POSITION ||
         command == SEESAW_ENCODER_DELTA)) {
      int32_t value;
      if (command == SEESAW_ENCODER_DELTA) {
        value = g_enc_delta[encoder_num];
      } else {
        value = g_enc_value[encoder_num];
      }
      Adafruit_seesawPeripheral_write32(value);
      // Reading either register clears the accumulated delta.
      g_enc_delta[encoder_num] = 0;
#if defined(ARDUINO_ARCH_STM32) && CONFIG_INTERRUPT
      Adafruit_seesawPeripheral_clearIRQ();
#endif
    }
  }
#endif

#if CONFIG_FHT && (defined(MEGATINYCORE) || defined(ARDUINO_ARCH_STM32))
  else if (base_cmd == SEESAW_SPECTRUM_BASE) {
    // TO DO: change to A/B/C/D results if we decide on FHT_N = 256.
    // That will require changes in Adafruit_Seesaw as well. Note that
    // this will only be possible if using an ATtiny part with 1K RAM
    // or better; won't fit on smaller devices.
    if (module_cmd == SEESAW_SPECTRUM_RESULTS_LOWER) {
#if defined(ARDUINO_ARCH_STM32)
      SeesawSTM32::snapshotSpectrum();
#endif
      Adafruit_seesawPeripheral_writeBuffer(
          Adafruit_seesawPeripheral_spectrumBuffer(), 32);
    } else if (module_cmd == SEESAW_SPECTRUM_RESULTS_UPPER) {
      Adafruit_seesawPeripheral_writeBuffer(
          Adafruit_seesawPeripheral_spectrumBuffer() + 32, 32);
    } else if (module_cmd == SEESAW_SPECTRUM_CHANNEL) {
      Adafruit_seesawPeripheral_write8(
          Adafruit_seesawPeripheral_readSpectrumChannel());
    } else if (module_cmd == SEESAW_SPECTRUM_RATE) {
      Adafruit_seesawPeripheral_write8(
          Adafruit_seesawPeripheral_readSpectrumRate());
    }
  }
#endif

#if CONFIG_UART
  else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS) {
      Adafruit_seesawPeripheral_write8(g_uart_status);
    } else if (module_cmd == SEESAW_SERCOM_INTEN) {
      Adafruit_seesawPeripheral_write8(g_uart_inten);
    } else if (module_cmd == SEESAW_SERCOM_BAUD) {
      Adafruit_seesawPeripheral_write32(g_uart_baud);
    } else if (module_cmd == SEESAW_SERCOM_DATA) {
      Adafruit_seesawPeripheral_write8(
          Adafruit_seesawPeripheral_readUARTData());
    }
  }
#endif

  else {
    SEESAW_DEBUG(F("Unhandled cmd 0x"));
    SEESAW_DEBUGLN(base_cmd, HEX);
  }
}
