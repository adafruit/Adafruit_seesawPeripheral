#if CONFIG_INTERRUPT
volatile uint32_t g_currentGPIO = 0, g_lastGPIO = 0;
#endif

#if USE_PINCHANGE_INTERRUPT
void Adafruit_seesawPeripheral_pinChangeDetect(void);

void Adafruit_seesawPeripheral_changedGPIO(void) {
  SEESAW_DEBUG(F("Change IRQ"));
  Adafruit_seesawPeripheral_pinChangeDetect();
}
#endif

#if CONFIG_INTERRUPT
void Adafruit_seesawPeripheral_pinChangeDetect(void) {

   uint32_t encoder_mask = 0;
#if CONFIG_ENCODER
    encoder_mask |= ENCODER0_INPUT_MASK;
    encoder_mask |= ENCODER1_INPUT_MASK; // these will be 0 if not used
    encoder_mask |= ENCODER2_INPUT_MASK;
    encoder_mask |= ENCODER3_INPUT_MASK;
#endif

  g_currentGPIO = Adafruit_seesawPeripheral_readBulk(g_irqGPIO | encoder_mask);
  uint32_t changedGPIO = (g_currentGPIO ^ g_lastGPIO) & g_irqGPIO;

  if (changedGPIO) {
    SEESAW_DEBUGLN(F("IRQ"));
    g_irqFlags |= (changedGPIO & g_irqGPIO); // flag the irq that changed
    Adafruit_seesawPeripheral_setIRQ();
  }

#if CONFIG_ENCODER
    uint32_t in = g_currentGPIO & encoder_mask;    
    
    for (uint8_t encodernum=0; encodernum<CONFIG_NUM_ENCODERS; encodernum++) {

      uint8_t enc_cur_pos = 0;
      // read in the encoder state first
      if (encodernum == 0) {
        enc_cur_pos |= ((BIT_IS_CLEAR(in, CONFIG_ENCODER0_A_PIN)) << 0) | ((BIT_IS_CLEAR(in, CONFIG_ENCODER0_B_PIN)) << 1);
        //SEESAW_DEBUG(F("GPIO 0x"));
        //SEESAW_DEBUGLN(g_currentGPIO, HEX);
        //SEESAW_DEBUG(F("IN 0x"));
        //SEESAW_DEBUGLN(in, HEX);
      }
#if defined(CONFIG_ENCODER1_A_PIN)
      if (encodernum == 1) {
        enc_cur_pos |= ((BIT_IS_CLEAR(in, CONFIG_ENCODER1_A_PIN)) << 0) | ((BIT_IS_CLEAR(in, CONFIG_ENCODER1_B_PIN)) << 1);      
      }
#endif
#if defined(CONFIG_ENCODER2_A_PIN)
      if (encodernum == 2) {
        enc_cur_pos |= ((BIT_IS_CLEAR(in, CONFIG_ENCODER2_A_PIN)) << 0) | ((BIT_IS_CLEAR(in, CONFIG_ENCODER2_B_PIN)) << 1);      
      }
#endif
#if defined(CONFIG_ENCODER3_A_PIN)
      if (encodernum == 3) {
        enc_cur_pos |= ((BIT_IS_CLEAR(in, CONFIG_ENCODER3_A_PIN)) << 0) | ((BIT_IS_CLEAR(in, CONFIG_ENCODER3_B_PIN)) << 1);      
      }
#endif

      Adafruit_seesawPeripheral_updateEncoder(encodernum, enc_cur_pos);
    }
#endif

  g_lastGPIO = g_currentGPIO;
}
#endif

void Adafruit_seesawPeripheral_run(void) {
#if CONFIG_INTERRUPT && ! USE_PINCHANGE_INTERRUPT
  // we dont .need. to use the IRQ system which takes a lot of flash and
  // doesn't uniquely identify pins anyways
  if (IRQ_debounce_cntr == 0) {
    Adafruit_seesawPeripheral_pinChangeDetect();
    IRQ_debounce_cntr = IRQ_DEBOUNCE_TICKS;
  }
#endif

#if CONFIG_INTERRUPT
  static uint32_t last_millis = 0;
  uint32_t now = millis();
  if (last_millis != now) {
    // one ms tick
    // tick down the gpio irq cheker
    if (IRQ_debounce_cntr)
      IRQ_debounce_cntr--;

    last_millis = now;
  }
#endif

#if CONFIG_FHT && defined(MEGATINYCORE)
  Adafruit_seesawPeripheral_runSpectrum();
#endif

#if CONFIG_UART
  Adafruit_seesawPeripheral_runUART();
#endif
  // delay(10);
}
