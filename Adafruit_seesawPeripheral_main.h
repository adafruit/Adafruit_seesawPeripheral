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

      int8_t enc_action = 0; // 1 or -1 if moved, sign is direction

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

      // if any rotation at all
      if (enc_cur_pos != g_enc_prev_pos[encodernum]) {
        SEESAW_DEBUG(F("Enc0 CurPos 0x"));
        SEESAW_DEBUGLN(enc_cur_pos, HEX);

        if (g_enc_prev_pos[encodernum] == 0x00) {
          // this is the first edge
          if (enc_cur_pos == 0x01) {
            g_enc_flags[encodernum] |= ENCODER_FLAG_FORW_EDGE1;
          }
          else if (enc_cur_pos == 0x02) {
            g_enc_flags[encodernum] |= ENCODER_FLAG_BACK_EDGE1;
          }
        }

        if (g_enc_prev_pos[encodernum] == 0x03) {
          // this is the second edge
          if (enc_cur_pos == 0x02) {
            g_enc_flags[encodernum] |= ENCODER_FLAG_FORW_EDGE2;
          }
          else if (enc_cur_pos == 0x01) {
            g_enc_flags[encodernum] |= ENCODER_FLAG_BACK_EDGE2;
          }
        }
        
        if ((enc_cur_pos == 0x03) && ! CONFIG_ENCODER_2TICKS) {
          // this is when the encoder is in the middle of a "step" of a 4-tick encoder
          g_enc_flags[encodernum] |= ENCODER_FLAG_MIDSTEP;
        }
        
        if ((enc_cur_pos == 0x00) || ((enc_cur_pos == 0x03) && CONFIG_ENCODER_2TICKS))
        {
          // this is when the encoder is in a 'rest' state
          
          // check the first and last edge
          // or maybe one edge is missing, if missing then require the middle state
          // this will reject bounces and false movements
          if ((g_enc_flags[encodernum] & ENCODER_FLAG_FORW_EDGE1) && 
              (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_FORW_EDGE2 | ENCODER_FLAG_MIDSTEP)))) {
            SEESAW_DEBUG(F("+1"));
            enc_action = 1;
          }
          else if ((g_enc_flags[encodernum] & ENCODER_FLAG_FORW_EDGE2) && 
                   (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_FORW_EDGE1 | ENCODER_FLAG_MIDSTEP)))) {
            SEESAW_DEBUG(F("+2"));
            enc_action = 1;
          }
          else if ((g_enc_flags[encodernum] & ENCODER_FLAG_BACK_EDGE1) && 
                   (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_BACK_EDGE2 | ENCODER_FLAG_MIDSTEP)))) {
            SEESAW_DEBUG(F("-1"));
            enc_action = -1;
          }
          else if ((g_enc_flags[encodernum] & ENCODER_FLAG_BACK_EDGE2) && 
                   (CONFIG_ENCODER_2TICKS || (g_enc_flags[encodernum] & (ENCODER_FLAG_BACK_EDGE1 | ENCODER_FLAG_MIDSTEP)))) {
            SEESAW_DEBUG(F("-2"));
            enc_action = -1;
          }
          
          g_enc_flags[encodernum] = 0; // reset for next time
        }
      }

      g_enc_prev_pos[encodernum] = enc_cur_pos;
      
      if(enc_action != 0){
        g_enc_value[encodernum] += enc_action;
        g_enc_delta[encodernum] += enc_action;        
      }
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

// Update soil readings
#if CONFIG_SOIL
  static uint32_t last_soil = 0;
  uint32_t now_soil = millis();
  if (now_soil - last_soil > g_soil_update_delay) {
    for (uint8_t s = 0; s < CONFIG_NUM_SOIL; s++) {
      if ((g_soil_samples[s] == 0)) continue;
      unsigned long t1, t2;
      unsigned long sum;
      int epin, spin;
      bool timeout;
      // configure pins
      switch(s) {
#if defined(CONFIG_SOIL0_EXC_PIN) && defined(CONFIG_SOIL0_SEN_PIN)
        case 0:
          epin = CONFIG_SOIL0_EXC_PIN;
          spin = CONFIG_SOIL0_SEN_PIN;
          break;
#endif
#if defined(CONFIG_SOIL1_EXC_PIN) && defined(CONFIG_SOIL1_SEN_PIN)
        case 1:
          epin = CONFIG_SOIL1_EXC_PIN;
          spin = CONFIG_SOIL1_SEN_PIN;
          break;
#endif
#if defined(CONFIG_SOIL2_EXC_PIN) && defined(CONFIG_SOIL2_SEN_PIN)
        case 2:
          epin = CONFIG_SOIL2_EXC_PIN;
          spin = CONFIG_SOIL2_SEN_PIN;
          break;
#endif
#if defined(CONFIG_SOIL3_EXC_PIN) && defined(CONFIG_SOIL3_SEN_PIN)
        case 3:
          epin = CONFIG_SOIL3_EXC_PIN;
          spin = CONFIG_SOIL3_SEN_PIN;
          break;
#endif
        default:
          continue;
      }
      // perform reading
      sum = 0;
      timeout = false;
      for (uint8_t n = 0; n < g_soil_samples[s]; n++) {
        // record charge time
        t1 = micros();
        digitalWrite(epin, HIGH);
        while (!digitalRead(spin)) {
          if ((micros() - t1) > g_soil_timeout[s]) {
            timeout = true;
            break;
          }
        }
        t2 = micros();
        // discharge
        digitalWrite(epin, LOW);
        delayMicroseconds(g_soil_xdelay[s] * (t2 - t1));
        // sum the deltas
        sum += t2 - t1;
        // give up if timed out
        if (timeout) break;
      }
      // store value, 0xFFFF = timed out
      if (timeout) {
        g_soil_value[s] = 0xFFFF;
      } else {
        sum /= g_soil_samples[s];
        if (sum == 0xFFFF) sum -= 1;
        g_soil_value[s] = uint16_t(sum);
      }
    }
    last_soil = now_soil;
  }
#endif

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
