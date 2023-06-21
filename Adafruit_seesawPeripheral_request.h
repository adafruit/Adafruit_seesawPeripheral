#if defined(ARDUINO_AVR_ATtiny806)
#define SEESAW_HW_ID 0x84
#elif defined(ARDUINO_AVR_ATtiny807)
#define SEESAW_HW_ID 0x85
#elif defined(ARDUINO_AVR_ATtiny816)
#define SEESAW_HW_ID 0x86
#elif defined(ARDUINO_AVR_ATtiny817)
#define SEESAW_HW_ID 0x87
#elif defined(ARDUINO_AVR_ATtiny1616)
#define SEESAW_HW_ID 0x88
#elif defined(ARDUINO_AVR_ATtiny1617)
#define SEESAW_HW_ID 0x89
#else
#error "Unsupported chip variant selected"
#endif

extern volatile uint32_t g_bufferedBulkGPIORead;

/***************************** data read */
void requestEvent(void) {
  // SEESAW_DEBUGLN(F("Requesting data"));
  uint8_t base_cmd = i2c_buffer[0];
  uint8_t module_cmd = i2c_buffer[1];

  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_HW_ID) {
      Wire.write(SEESAW_HW_ID); // instant reply
    }
    if (module_cmd == SEESAW_STATUS_VERSION) {
      Adafruit_seesawPeripheral_write32(CONFIG_VERSION | DATE_CODE); // instant reply
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
      Wire.write(g_bufferedADCRead >> 8);
      Wire.write(g_bufferedADCRead);
    } else if (module_cmd == SEESAW_ADC_STATUS) {
      Wire.write(g_adcStatus);
    }
  }
#endif

#if CONFIG_EEPROM
  else if (base_cmd == SEESAW_EEPROM_BASE) {
    Wire.write(EEPROM.read(module_cmd % EEPROM.length()));
  }
#endif

#if CONFIG_ENCODER
  else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t encoder_num = 0;
    if ((module_cmd & 0xF0) == SEESAW_ENCODER_POSITION) {
      encoder_num = module_cmd & 0x0F;
      if (encoder_num < CONFIG_NUM_ENCODERS){
        Adafruit_seesawPeripheral_write32(g_enc_value[encoder_num]);
        g_enc_delta[encoder_num] = 0;
      }
    }
    else if ((module_cmd & 0xF0) == SEESAW_ENCODER_DELTA) {
      encoder_num = module_cmd & 0x0F;
      if (encoder_num < CONFIG_NUM_ENCODERS){
        Adafruit_seesawPeripheral_write32(g_enc_delta[encoder_num]);
        g_enc_delta[encoder_num] = 0;
      }
    }
  }
#endif

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

#if CONFIG_UART
  else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS) {
      Wire.write(g_uart_status);
    } else if (module_cmd == SEESAW_SERCOM_INTEN) {
      Wire.write(g_uart_inten);
    } else if (module_cmd == SEESAW_SERCOM_BAUD) {
      Wire.write((g_uart_baud >> 24) & 0xFF);
      Wire.write((g_uart_baud >> 16) & 0xFF);
      Wire.write((g_uart_baud >> 8) & 0xFF);
      Wire.write(g_uart_baud & 0xFF);
    } else if (module_cmd == SEESAW_SERCOM_DATA) {
      Wire.write(CONFIG_UART_SERCOM.read());
    }
  }
#endif

  else {
    SEESAW_DEBUG(F("Unhandled cmd 0x"));
    SEESAW_DEBUGLN(base_cmd, HEX);
  }
}
