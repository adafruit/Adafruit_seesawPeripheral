
/***************************** data read */
void requestEvent(void) {
  uint32_t temp = 0;
  
  SEESAW_DEBUGLN(F("Requesting data"));

  base_cmd = i2c_buffer[0];
  module_cmd = i2c_buffer[1];

  if (base_cmd == SEESAW_STATUS_BASE) {
    if (module_cmd == SEESAW_STATUS_HW_ID) {
       Wire.write(SEESAW_HW_ID_CODE);
    }
    if (module_cmd == SEESAW_STATUS_VERSION) {
      Adafruit_seesawPeripheral_write32(CONFIG_VERSION);
    }
  }
  else if (base_cmd == SEESAW_GPIO_BASE) {
    temp = 0x0;
    if (module_cmd == SEESAW_GPIO_BULK) {
       Adafruit_seesawPeripheral_write32(Adafruit_seesawPeripheral_readBulk(VALID_GPIO));
    }
#if CONFIG_INTERRUPT
    else if (module_cmd == SEESAW_GPIO_INTFLAG) {
       Adafruit_seesawPeripheral_write32(g_irqFlags);
       g_irqFlags = 0; // reading the flags clears them
    }
#endif
  }
}
