
volatile uint32_t g_currentGPIO = 0, g_lastGPIO = 0;


#if USE_PINCHANGE_INTERRUPT
void Adafruit_seesawPeripheral_changedGPIO(void) {
  SEESAW_DEBUG(F("Change IRQ"));
  Adafruit_seesawPeripheral_pinChangeDetect()
}
#endif

#if CONFIG_INTERRUPT
void Adafruit_seesawPeripheral_pinChangeDetect(void) {
  g_currentGPIO = Adafruit_seesawPeripheral_readBulk();
  uint32_t changedGPIO = (g_currentGPIO ^ g_lastGPIO) & g_irqGPIO;

  if (changedGPIO) {
    SEESAW_DEBUGLN(F("IRQ"));
    IRQ_pulse_cntr = IRQ_PULSE_TICKS;
    g_irqFlags |= (changedGPIO & g_irqGPIO);  // flag the irq that changed
    digitalWrite(CONFIG_INTERRUPT_PIN, LOW);
    pinMode(CONFIG_INTERRUPT_PIN, OUTPUT);
  }

  g_lastGPIO = g_currentGPIO;
}
#endif

void Adafruit_seesawPeripheral_run(void) {
#if CONFIG_INTERRUPT && ! USE_PINCHANGE_INTERRUPT
  // we dont .need. to use the IRQ system which takes a lot of flash and doesn't uniquely
  // identify pins anyways
  cli();
  Adafruit_seesawPeripheral_pinChangeDetect();
  sei();
#endif

  static uint32_t last_millis = 0;
  if (last_millis != millis()) {
    // one ms tick

#if CONFIG_INTERRUPT
    // tick down the pulse width
    if (IRQ_pulse_cntr) 
      IRQ_pulse_cntr--;
    // time to turn off the IRQ pin?
    if (!IRQ_pulse_cntr) {
      pinMode(CONFIG_INTERRUPT_PIN, INPUT_PULLUP); // open-drainish
    }
#endif
  }
  last_millis = millis();
  //delay(10);
}
