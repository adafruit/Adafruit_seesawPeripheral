/***************************** data write */

void receiveEvent(int howMany) {
  SEESAW_DEBUG(F("Received "));
  SEESAW_DEBUG(howMany);
  SEESAW_DEBUG(F(" bytes:"));

  if ((uint32_t)howMany > sizeof(i2c_buffer)) {
    SEESAW_DEBUGLN();
    return;
  }
  for (uint8_t i=0; i<howMany; i++) {
    i2c_buffer[i] = Wire.read();
    SEESAW_DEBUG(F("0x"));
    SEESAW_DEBUG(i2c_buffer[i], HEX);
    SEESAW_DEBUG(F(" "));
  }
  SEESAW_DEBUG("\n");

  uint8_t base_cmd = i2c_buffer[0];
  uint8_t module_cmd = i2c_buffer[1];
  
  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_SWRST) {
      Adafruit_seesawPeripheral_reset();
      SEESAW_DEBUG(F("Resetting"));
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
              }
              else if (module_cmd == SEESAW_GPIO_INTENCLR) {
                g_irqGPIO &= ~(1UL << pin);
                SEESAW_DEBUGLN(F(" INTCLR"));
              }
#endif
            }
          }
    }
  }

#if CONFIG_PWM
  else if (base_cmd == SEESAW_TIMER_BASE) {
    uint8_t pin = i2c_buffer[2];
    uint16_t value = i2c_buffer[3];
    value <<= 8;
    value |= i2c_buffer[4];
    if (! (VALID_PWM & (1UL << pin))) {
      g_pwmStatus = 0x1; // error, invalid pin!
    } else if (module_cmd == SEESAW_TIMER_PWM) {
      // its valid!
      value >>= 8;  // we only support 8 bit analogwrites
      SEESAW_DEBUG(F("PWM "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);
      
      pinMode(pin, OUTPUT);
      analogWrite(pin, value);
      g_pwmStatus = 0x0;
    }
    else if (module_cmd == SEESAW_TIMER_FREQ) {
      SEESAW_DEBUG(F("Freq "));
      SEESAW_DEBUG(pin);
      SEESAW_DEBUG(F(": "));
      SEESAW_DEBUGLN(value);
      tone(pin, value);
      g_pwmStatus = 0x0;
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
      SEESAW_DEBUGLN(F("Neo show!"));
      pinMode(g_neopixel_pin, OUTPUT);
      tinyNeoPixel_show(g_neopixel_pin, g_neopixel_bufsize, (uint8_t *)g_neopixel_buf);
    }
  }
#endif
}
