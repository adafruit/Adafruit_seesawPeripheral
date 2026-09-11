/*!
 * @file Adafruit_seesawPeripheral_request.h
 * One register-response implementation for AVR and STM32.
 */

/*! Return the selected register through Wire, preserving read-to-clear state.
 */
void requestEvent(void) {
#if defined(ARDUINO_ARCH_STM32)
  using namespace SeesawSTM32;
  uint8_t base_cmd = selectedBase;
  uint8_t module_cmd = selectedRegister;
#else
  uint8_t base_cmd = i2c_buffer[0];
  uint8_t module_cmd = i2c_buffer[1];
#endif
#if CONFIG_SPI && defined(ARDUINO_ARCH_STM32)
  if (base_cmd == spiBase) {
    spiRequest(module_cmd);
    return;
  }
#endif
  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_HW_ID)
      Wire.write((uint8_t)SEESAW_HW_ID);
    else if (module_cmd == SEESAW_STATUS_VERSION)
      Adafruit_seesawPeripheral_write32(CONFIG_VERSION);
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
#if CONFIG_SPI && defined(ARDUINO_ARCH_STM32)
      options |= 1UL << spiBase;
#endif
      Adafruit_seesawPeripheral_write32(options);
    } else
      Wire.write((uint8_t)0);
  } else if (base_cmd == SEESAW_GPIO_BASE) {
    if (module_cmd == SEESAW_GPIO_BULK) {
#if defined(ARDUINO_ARCH_STM32)
      Adafruit_seesawPeripheral_write32(readGPIO());
#else
      Adafruit_seesawPeripheral_write32(g_bufferedBulkGPIORead);
#endif
#if CONFIG_INTERRUPT || defined(ARDUINO_ARCH_STM32)
      g_irqFlags = 0;
#if defined(ARDUINO_ARCH_STM32)
      updateIRQ();
#else
      Adafruit_seesawPeripheral_clearIRQ();
#endif
#endif
    }
#if CONFIG_INTERRUPT || defined(ARDUINO_ARCH_STM32)
    else if (module_cmd == SEESAW_GPIO_INTFLAG) {
      Adafruit_seesawPeripheral_write32(g_irqFlags);
      g_irqFlags = 0;
#if defined(ARDUINO_ARCH_STM32)
      updateIRQ();
#else
      Adafruit_seesawPeripheral_clearIRQ();
#endif
    }
#endif
#if CONFIG_ADC
  } else if (base_cmd == SEESAW_ADC_BASE) {
    if (module_cmd >= SEESAW_ADC_CHANNEL_OFFSET)
      Adafruit_seesawPeripheral_write16(g_bufferedADCRead);
    else if (module_cmd == SEESAW_ADC_STATUS)
      Wire.write(g_adcStatus);
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  } else if (base_cmd == SEESAW_TIMER_BASE) {
    Wire.write(g_pwmStatus);
#endif
#if CONFIG_EEPROM
  } else if (base_cmd == SEESAW_EEPROM_BASE) {
#if defined(ARDUINO_ARCH_STM32)
    uint8_t value = 0xFF;
    if (eepromSafe)
      value = *((const uint8_t *)eepromAddress + module_cmd);
    Wire.write(value);
#else
    Wire.write(EEPROM.read(module_cmd % EEPROM.length()));
#endif
#endif
#if CONFIG_NEOPIXEL
  } else if (base_cmd == SEESAW_NEOPIXEL_BASE) {
    Wire.write(g_neopixel_status);
#endif
#if CONFIG_ENCODER
  } else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t index = module_cmd & 0x0F;
    uint8_t command = module_cmd & 0xF0;
    if (index < CONFIG_NUM_ENCODERS && (command == SEESAW_ENCODER_POSITION ||
                                        command == SEESAW_ENCODER_DELTA)) {
      int32_t value = g_enc_value[index];
      if (command == SEESAW_ENCODER_DELTA)
        value = g_enc_delta[index];
      Adafruit_seesawPeripheral_write32(value);
      g_enc_delta[index] = 0;
#if defined(ARDUINO_ARCH_STM32)
      updateIRQ();
#endif
    }
#endif
#if CONFIG_UART
  } else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS) {
#if defined(ARDUINO_ARCH_STM32)
      g_uart_status = CONFIG_UART_SERCOM.available() ? 2 : 0;
#endif
      Wire.write(g_uart_status);
    } else if (module_cmd == SEESAW_SERCOM_INTEN)
      Wire.write(g_uart_inten);
    else if (module_cmd == SEESAW_SERCOM_BAUD)
      Adafruit_seesawPeripheral_write32(g_uart_baud);
    else if (module_cmd == SEESAW_SERCOM_DATA)
      Wire.write((uint8_t)CONFIG_UART_SERCOM.read());
#endif
  }
#if CONFIG_FHT && defined(MEGATINYCORE)
  else if (base_cmd == SEESAW_SPECTRUM_BASE) {
    // TO DO: change to A/B/C/D results if we decide on FHT_N = 256.
    // That will require changes in Adafruit_Seesaw as well. Note that
    // this will only be possible if using an ATtiny part with 1K RAM
    // or better; won't fit on smaller devices.
    if (module_cmd == SEESAW_SPECTRUM_RESULTS_LOWER) {
      Wire.write(fht_log_out, 32);
    } else if (module_cmd == SEESAW_SPECTRUM_RESULTS_UPPER) {
      Wire.write(&fht_log_out[32], 32);
    } else if (module_cmd == SEESAW_SPECTRUM_CHANNEL) {
      // TO DO: this should re-map the current MUXPOS setting to the same
      // channel mapping as is used in Adafruit_seesawPeripheral_receive.h --
      // see notes over there. Prob just two values. For now though, for the
      // sake of initial testing, this just returns the raw MUXPOS setting.
      Wire.write(ADC0.MUXPOS); // Return current ADC channel
    } else if (module_cmd == SEESAW_SPECTRUM_RATE) {
      Wire.write(ADC0.SAMPCTRL); // Return current sample rate index
    }
  }
#endif

  else
    Wire.write((uint8_t)0);
}
