#if CONFIG_FHT && defined(MEGATINYCORE)
#include "fht_tables.h"
#endif

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
  g_currentGPIO = Adafruit_seesawPeripheral_readBulk(g_irqGPIO);
  uint32_t changedGPIO = (g_currentGPIO ^ g_lastGPIO) & g_irqGPIO;

  if (changedGPIO) {
    SEESAW_DEBUGLN(F("IRQ"));
    g_irqFlags |= (changedGPIO & g_irqGPIO); // flag the irq that changed
    Adafruit_seesawPeripheral_setIRQ();
  }

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
  while (ADC0.INTCTRL & ADC_RESRDY_bm)
    ; // Wait for sampling to finish

  fht_window();
  fht_reorder();
  fht_run();

  // I2C interrupts are temporarily disabled so we don't return spectrum
  // data in the middle of it being processed. If an I2C event occurs,
  // the corresponding interrupt flag will still get set, it just won't
  // act on it until later when the interrupts are re-enabled.
  // Up to this point though, everything's happening in fht_input[].
  TWI0.SCTRLA &= ~(TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm);

  // FHT has various output modes, after much testing settled on this...
  fht_mag_log(); // -> fht_log_out[], if using this, define LOG_OUT to 1

  // Resume sampling at earliest opportunity...
  fht_counter = 0;
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt

  // Process and/or dump FHT output
  for (uint8_t i = 0; i < FHT_N / 2; i++) {
    if (fht_log_out[i] > noise[i]) {
      fht_log_out[i] -= noise[i];
      if (fht_log_out[i] < peak[i]) {
        // Scale column to make up for range lost to noise subtraction.
        // Initially thought some additional per-column scaling would be
        // needed to boost the highs, but when testing with pure tones
        // the response across the graph looks reasonably linear-ish
        // enough, the lower peaks at the top are really just a function
        // of how music is. Host-side code can do some auto-scale
        // massaging of the data if really desired (a little frame-to-
        // frame filtering there looks nice anyway).
        fht_log_out[i] = (fht_log_out[i] * scale[i]) >> 8;
      } else {
        fht_log_out[i] = 255; // At or above peak
      }
    } else {
      fht_log_out[i] = 0; // At or below noise threshold
    }
    // Note to future self: noise and peak are a function of the
    // microphone, and they do seem to vary with sampling rate.
    // There's ample flash remaining, one possibility is to use
    // different tables depending on the active rate setting.
    // Would want to record those on final hardware.
  }

  // Re-enable I2C interrupts -- safe to issue spectrum data
  TWI0.SCTRLA |= TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm;
#endif // end CONFIG_FHT

#if CONFIG_UART
  if (g_uart_inten) {
    if (CONFIG_UART_SERCOM.available()) {
      Adafruit_seesawPeripheral_setIRQ();
    } else {
      Adafruit_seesawPeripheral_clearIRQ();
    }
  }
  if (g_uart_tx_len > 0) {
    CONFIG_UART_SERCOM.write((char *)g_uart_buf, (size_t)g_uart_tx_len);
    g_uart_tx_len = 0;
  }
#endif
  // delay(10);
}
