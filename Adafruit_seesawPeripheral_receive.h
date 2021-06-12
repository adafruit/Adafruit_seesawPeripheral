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

  base_cmd = i2c_buffer[0];
  module_cmd = i2c_buffer[1];
  
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
}
