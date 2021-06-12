void Adafruit_seesawPeripheral_run(void) {

  static uint32_t currentGPIO = 0, lastGPIO = 0;
  
#if CONFIG_INTERRUPT
  static uint8_t IRQ_pulse_cntr = 0;

  // time to turn off the IRQ pin?
  if (!IRQ_pulse_cntr) {
    digitalWrite(CONFIG_INTERRUPT_PIN, LOW);
  }
#endif

  cli();
  currentGPIO = Adafruit_seesawPeripheral_readBulk();
  //SEESAW_DEBUG('.');
  //SEESAW_DEBUG(F("CurrGPIO: 0x"));
  //SEESAW_DEBUGLN(currentGPIO, HEX);

  uint32_t changedGPIO = currentGPIO ^ lastGPIO;
  if (changedGPIO) {
    // something changed!
    SEESAW_DEBUG(F("Changed 0x"));
    SEESAW_DEBUGLN(changedGPIO, HEX);
#if CONFIG_INTERRUPT
    if (changedGPIO & g_irqGPIO) {
      SEESAW_DEBUGLN(F("IRQ"));
      IRQ_pulse_cntr = IRQ_PULSE_TICKS;
      g_irqFlags |= (changedGPIO & g_irqGPIO);  // flag the irq that changed
    }
    digitalWrite(CONFIG_INTERRUPT_PIN, HIGH);
#endif
  }
  lastGPIO = currentGPIO;

  sei();

#if CONFIG_INTERRUPT
  if (IRQ_pulse_cntr) IRQ_pulse_cntr--;
#endif

  // one tick
  delay(100);
}
