volatile uint32_t g_bufferedBulkGPIORead = 0;
volatile uint16_t g_bufferedADCRead = 0;

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

/***************************** data write */

void receiveEvent(int howMany) {
  SEESAW_DEBUG(F("Received "));
  SEESAW_DEBUG(howMany);
  SEESAW_DEBUG(F(" bytes:"));
  for (uint8_t i=howMany; i<sizeof(i2c_buffer); i++) {
    i2c_buffer[i] = 0;
  }

  if ((uint32_t)howMany > sizeof(i2c_buffer)) {
    SEESAW_DEBUGLN();
    return;
  }
  for (uint8_t i=0; i<howMany; i++) {
    i2c_buffer[i] = Wire.read();
    //SEESAW_DEBUG(F("0x"));
    //SEESAW_DEBUG(i2c_buffer[i], HEX);
    //SEESAW_DEBUG(F(" "));
  }
  //SEESAW_DEBUG("\n");

  uint8_t base_cmd = i2c_buffer[0];
  uint8_t module_cmd = i2c_buffer[1];

  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_SWRST) {
      Adafruit_seesawPeripheral_reset();
      SEESAW_DEBUGLN(F("Resetting"));
    }
  }
  else if (base_cmd == SEESAW_GPIO_BASE) {
    uint32_t temp;
    temp = i2c_buffer[2];
    temp <<= 8;
    temp |= i2c_buffer[3];
    temp <<= 8;
    temp |= i2c_buffer[4];
    temp <<= 8;
    temp |= i2c_buffer[5];

    switch (module_cmd) {
      case SEESAW_GPIO_BULK:
        if (howMany == 2) {
          // we're about to request the data next so we'll do the read now
          g_bufferedBulkGPIORead = Adafruit_seesawPeripheral_readBulk(VALID_GPIO);
        } else {
          //pinMode(1, OUTPUT);
          //digitalWriteFast(1, HIGH);
          // otherwise, we are writing bulk data!
          uint32_t pinmask = 0x1;
          for (uint8_t pin=0; pin<32; pin++) {
            if (VALID_GPIO & pinmask) {
              digitalWrite(pin, (temp >> pin) & 0x1);
            }
            pinmask <<= 1;
          }
         // digitalWriteFast(1, LOW);
        }
        break;
      case SEESAW_GPIO_DIRSET_BULK:
      case SEESAW_GPIO_DIRCLR_BULK:
      case SEESAW_GPIO_BULK_SET:
      case SEESAW_GPIO_BULK_CLR:
      case SEESAW_GPIO_PULLENSET:
      case SEESAW_GPIO_PULLENCLR:
      case SEESAW_GPIO_INTENSET:
          temp &= VALID_GPIO;
          for (uint8_t pin=0; pin<32; pin++) {
            if ((temp >> pin) & 0x1) {
              SEESAW_DEBUG(F("Set pin "));
              SEESAW_DEBUG(pin);

              if (module_cmd == SEESAW_GPIO_DIRSET_BULK) {
                pinMode(pin, OUTPUT);
                SEESAW_DEBUGLN(F(" OUTPUT"));
              }
              else if (module_cmd == SEESAW_GPIO_DIRCLR_BULK) {
                pinMode(pin, INPUT);
                SEESAW_DEBUGLN(F(" INPUT"));
              }
              else if (module_cmd == SEESAW_GPIO_BULK_SET) {
                digitalWrite(pin, 1);
                SEESAW_DEBUGLN(F(" HIGH"));
              }
              else if (module_cmd == SEESAW_GPIO_BULK_CLR) {
                digitalWrite(pin, 0);
                SEESAW_DEBUGLN(F(" LOW"));
              }
              else if (module_cmd == SEESAW_GPIO_PULLENSET) {
                pinMode(pin, INPUT_PULLUP);
                SEESAW_DEBUGLN(F(" PULL"));
              }
              else if (module_cmd == SEESAW_GPIO_PULLENCLR) {
                pinMode(pin, INPUT);
                SEESAW_DEBUGLN(F(" NOPULL"));
              }
#if CONFIG_INTERRUPT
              else if (module_cmd == SEESAW_GPIO_INTENSET) {
                g_irqGPIO |= 1UL << pin;
                SEESAW_DEBUGLN(F(" INTEN"));
#if USE_PINCHANGE_INTERRUPT
                attachInterrupt(digitalPinToInterrupt(pin), Adafruit_seesawPeripheral_changedGPIO, CHANGE);
#endif
              }
              else if (module_cmd == SEESAW_GPIO_INTENCLR) {
                g_irqGPIO &= ~(1UL << pin);
                SEESAW_DEBUGLN(F(" INTCLR"));
#if USE_PINCHANGE_INTERRUPT
                detachInterrupt(digitalPinToInterrupt(pin));
#endif
              }
#endif
            }
          }
    }
  }

#if CONFIG_ADC
  else if (base_cmd == SEESAW_ADC_BASE) {
    if (module_cmd >= SEESAW_ADC_CHANNEL_OFFSET) {
      uint8_t adcpin = module_cmd - SEESAW_ADC_CHANNEL_OFFSET;
      if (!((VALID_ADC) & (1UL << adcpin))) {
        g_adcStatus = 0x1; // error, invalid pin!
      } else {
        // its valid!
        SEESAW_DEBUG(F("ADC read "));
        SEESAW_DEBUG(adcpin);
        SEESAW_DEBUG(F(": "));
        g_bufferedADCRead = analogRead(adcpin);
        SEESAW_DEBUGLN(g_bufferedADCRead);
        g_adcStatus = 0x0;
      }
    }
  }
#endif


#if (CONFIG_PWM | CONFIG_PWM_16BIT)
  else if (base_cmd == SEESAW_TIMER_BASE) {
    uint8_t pin = i2c_buffer[2];
    uint16_t value = i2c_buffer[3];
    value <<= 8;
    value |= i2c_buffer[4];
    if (! (VALID_PWM & (1UL << pin))) {
      g_pwmStatus = 0x1; // error, invalid pin!
    } else if (module_cmd == SEESAW_TIMER_PWM) {
      // its valid!
#if CONFIG_PWM
      value >>= 8;  // we only support 8 bit analogwrites
#endif
      SEESAW_DEBUG(F("PWM "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);

      pinMode(pin, OUTPUT);
#if CONFIG_PWM
      analogWrite(pin, value);
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
      g_pwmStatus = 0x0;
    }
    else if (module_cmd == SEESAW_TIMER_FREQ) {
      SEESAW_DEBUG(F("Freq "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);
#if CONFIG_PWM
      tone(pin, value);
#elif CONFIG_PWM_16BIT
      pinMode(pin, OUTPUT);
      // set frequency via CLKSEL and PER based on F_CPU
      // NOTE: changes all PWM outputs
      uint8_t clksel = 0;
      unsigned long period = (F_CPU / value);
      while (period > 65536 && clksel < 7) {
        clksel++;
        period = period >> (clksel > 4 ? 2 : 1);
      }
      TCA0.SINGLE.CTRLA = (clksel << 1) | TCA_SINGLE_ENABLE_bm;
      TCA0.SINGLE.PER = period;
      g_pwmStatus = 0x0;
#endif
    }
  }
#endif

#if CONFIG_NEOPIXEL
  else if (base_cmd == SEESAW_NEOPIXEL_BASE) {
    if (module_cmd == SEESAW_NEOPIXEL_SPEED) {
      // we only support 800khz anyways
    }
    else if (module_cmd == SEESAW_NEOPIXEL_BUF_LENGTH) {
      uint16_t value = i2c_buffer[2];
      value <<= 8;
      value |= i2c_buffer[3];
      // dont let it be larger than the internal buffer, of course
      g_neopixel_bufsize = min((uint16_t)CONFIG_NEOPIXEL_BUF_MAX, value);
      SEESAW_DEBUG(F("Neolen "));
      SEESAW_DEBUGLN(g_neopixel_bufsize);
    }
    if (module_cmd == SEESAW_NEOPIXEL_PIN) {
      g_neopixel_pin = i2c_buffer[2];
      SEESAW_DEBUG(F("Neopin "));
      SEESAW_DEBUGLN(g_neopixel_pin);
    }
    if (module_cmd == SEESAW_NEOPIXEL_BUF) {
      uint16_t offset = i2c_buffer[2];
      offset <<= 8;
      offset |= i2c_buffer[3];

      for (uint8_t i=0; i<howMany-4; i++) {
        if (offset+i < CONFIG_NEOPIXEL_BUF_MAX) {
          g_neopixel_buf[offset+i] = i2c_buffer[4+i];
        }
      }
    }
    if (module_cmd == SEESAW_NEOPIXEL_SHOW) {
      //SEESAW_DEBUGLN(F("Neo show!"));
      pinMode(g_neopixel_pin, OUTPUT);
      tinyNeoPixel_show(g_neopixel_pin, g_neopixel_bufsize, (uint8_t *)g_neopixel_buf);
    }
  }
#endif

#if CONFIG_ENCODER
  else if (base_cmd == SEESAW_ENCODER_BASE) {
    uint8_t encoder_num;
    if ((module_cmd & 0xF0) ==  SEESAW_ENCODER_INTENSET) {
      encoder_num = module_cmd & 0x0F;
      if (encoder_num < CONFIG_NUM_ENCODERS) {
        if (encoder_num == 0) {
          g_irqGPIO |= ENCODER0_INPUT_MASK;
        }
        if (encoder_num == 1) {
          g_irqGPIO |= ENCODER1_INPUT_MASK;
        }
        if (encoder_num == 2) {
          g_irqGPIO |= ENCODER2_INPUT_MASK;
        }
        if (encoder_num == 3) {
          g_irqGPIO |= ENCODER3_INPUT_MASK;
        }
      }
    }
    else if ((module_cmd & 0xF0) ==  SEESAW_ENCODER_INTENCLR) {
      encoder_num = module_cmd & 0x0F;
      if (encoder_num < CONFIG_NUM_ENCODERS) {
        if (encoder_num == 0) {
          g_irqGPIO &= ~ENCODER0_INPUT_MASK;
        }
        if (encoder_num == 1) {
          g_irqGPIO &= ~ENCODER1_INPUT_MASK;
        }
        if (encoder_num == 2) {
          g_irqGPIO &= ~ENCODER2_INPUT_MASK;
        }
        if (encoder_num == 3) {
          g_irqGPIO &= ~ENCODER3_INPUT_MASK;
        }
      }
    }
    else if ((module_cmd & 0xF0) ==  SEESAW_ENCODER_POSITION) {
      if (howMany == 6) { 
      encoder_num = module_cmd & 0x0F;
        if (encoder_num < CONFIG_NUM_ENCODERS){
          uint32_t value = i2c_buffer[2];
          value <<= 8;
          value |= i2c_buffer[3];
          value <<= 8;
          value |= i2c_buffer[4];
          value <<=8;
          value |= i2c_buffer[5];
          g_enc_value[encoder_num] = value;
        }
      }
    }
  }
#endif


#if CONFIG_EEPROM
  else if (base_cmd == SEESAW_EEPROM_BASE) {
    // special case for 1 byte at -1 (i2c addr)
    if ((module_cmd == 0xFF) && (howMany == 3)) {
      EEPROM.write(EEPROM.length()-1, i2c_buffer[2]);
    } else {
      // write the data
      for (uint8_t i=0; i<howMany-2; i++) {
        if ((module_cmd+i) < EEPROM.length()) {
          EEPROM.write(module_cmd+i, i2c_buffer[2+i]);
          SEESAW_DEBUG(F("EEP $"));
          SEESAW_DEBUG(module_cmd+i, HEX);
          SEESAW_DEBUG(F(" <- 0x"));
          SEESAW_DEBUGLN(i2c_buffer[2+i], HEX);
        }
      }
    }
  }
#endif

#if CONFIG_UART
  else if (base_cmd == SEESAW_SERCOM0_BASE) {
    if (module_cmd == SEESAW_SERCOM_STATUS) {
      if (CONFIG_UART_SERCOM.available()) {
        g_uart_status |= 0x02;  // set DATA_RDY bit
      } else {
        g_uart_status &= ~0x02; // clear DATA_RDY bit
      }
    } else if ((module_cmd == SEESAW_SERCOM_INTEN) && (howMany == 3)){
      // writing 0 to this register has no effect
      if (i2c_buffer[2] & 0x01) {
        g_uart_inten = 1;
      }
    } else if ((module_cmd == SEESAW_SERCOM_INTENCLR) && (howMany == 3)){
      // writing 0 to this register has no effect
      if (i2c_buffer[2] & 0x01) {
        g_uart_inten = 0;
      }
    } else if ((module_cmd == SEESAW_SERCOM_BAUD) && (howMany == 6)){
      uint32_t newBaud =
        uint32_t(i2c_buffer[2]) << 24 |
        uint32_t(i2c_buffer[3]) << 16 |
        uint32_t(i2c_buffer[4]) << 8 |
        i2c_buffer[5];
      if (newBaud != g_uart_baud) {
        g_uart_baud = newBaud;
        CONFIG_UART_SERCOM.end();
        CONFIG_UART_SERCOM.begin(g_uart_baud);
      }
    } else if (module_cmd ==  SEESAW_SERCOM_DATA) {
      g_uart_tx_len = howMany - 2;
      if (g_uart_tx_len <= CONFIG_UART_BUF_MAX) {
        for (uint8_t i=0; i<g_uart_tx_len; i++) {
          g_uart_buf[i] = i2c_buffer[i+2];
        }
      } else {
        g_uart_tx_len = 0;
      }
    }
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
  else if (base_cmd == SEESAW_SPECTRUM_BASE) {
    if ((module_cmd == SEESAW_SPECTRUM_RATE) && (howMany == 3)) {
      ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready IRQ
      uint8_t rate = i2c_buffer[2];            // Requested rate index
      if (rate > 31) rate = 31;                // Clip rate between 0-31
      ADC0.SAMPCTRL = rate & 31;               // Set ADC sample control
      restart_sampling();                      // Purge recording, start over
    } else if ((module_cmd == SEESAW_SPECTRUM_CHANNEL) && (howMany == 3)) {
      ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready IRQ
      uint8_t channel = i2c_buffer[2];         // Requested ADC channel
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
