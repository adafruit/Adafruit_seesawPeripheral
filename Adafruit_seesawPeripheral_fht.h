#if CONFIG_FHT && defined(MEGATINYCORE)


#if 0


#ifdef STEMMA


// These are virtual device registers when accessing this as an
// I2C peripheral. AVR only allows transferring 32 bytes at a
// time, so the spectrum is divided into "lower" and "upper"
// which much be separately requested from registers 0 and 1,
// respectively. If some future device supports a larger spectrum,
// can add additional "buckets" working upward from here.
#define REG_LOWER_SPECTRUM 0
#define REG_UPPER_SPECTRUM 1
// REG_IDLE is sort of a no-op indication if we somehow end up
// in the requestEvent() callback unexpectedly.
// Currently there is one configurable setting, via register
// REG_RATE. If more configurables are added in the future, work
// downward from here to avoid bucket & configurables collision.
#define REG_IDLE 255
#define REG_RATE 254

static volatile uint8_t reg = REG_IDLE; // Used by I2C callbacks

// Called when data is requested from the device.
static void requestEvent(void) {
  switch (reg) {
    case REG_LOWER_SPECTRUM:
      WIRE.write(fht_log_out, 32);
      break;
    case REG_UPPER_SPECTRUM:
      WIRE.write(&fht_log_out[32], 32);
      break;
    case REG_RATE:
      WIRE.write(ADC0.SAMPCTRL);
      break;
  }
  reg = REG_IDLE;
}

// Called when data is received by the device.
static void receiveEvent(int howMany) {
  reg = WIRE.read();
  if ((reg == REG_RATE) && (howMany > 1)) {  // Setting rate?
    ADC0.INTCTRL &= ~ADC_RESRDY_bm;          // Disable result-ready interrupt
    uint8_t rate = min(WIRE.read(), 31);     // Read rate
    howMany--;                               // Subtract 1 byte for rate
    if (rate > 31) rate = 31;                // Clip rate between 0-31
    ADC0.SAMPCTRL = rate & 31;               // Set ADC sample control
    for(uint8_t i=0; i<3; i++) {             // Discard initial readings
      while(!ADC0.INTFLAGS & ADC_RESRDY_bm); // In the INTFLAG register,
      ADC0.INTFLAGS |= ADC_RESRDY_bm;        // setting bit clears flag!
      // (ADC is still free-running and will set RESRDY bit,
      // it's just not triggering interrupts right now.)
    }
    counter = 0;                   // Restart at beginning of buffer
    ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt
  }
  while (--howMany) (void)WIRE.read(); // Discard any extraneous data
}

#endif // end STEMMA

void loop() {
  while (ADC0.INTCTRL & ADC_RESRDY_bm); // Wait for sampling to finish

  fht_window();
  fht_reorder();
  fht_run();

#ifdef STEMMA
  // I2C interrupts are temporarily disabled so we don't return spectrum
  // data in the middle of it being processed. If an I2C event occurs,
  // the corresponding interrupt flag will still get set, it just won't
  // act on it until later when the interrupts are re-enabled.
  // Up to this point though, everything's happening in fht_input[].
  TWI0.SCTRLA &= ~(TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm);
#endif

  // FHT has various output modes, after much testing settled on this...
  fht_mag_log(); // -> fht_log_out[], if using this, define LOG_OUT to 1

  // Resume sampling at earliest opportunity...
  counter = 0;
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt

  // Process and/or dump FHT output
  for (uint8_t i = 0; i < 64; i++) {
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
#ifndef STEMMA
    Serial.print(fht_log_out[i]);
    Serial.write(' ');
#endif
  }
#ifdef STEMMA
  // Re-enable I2C interrupts -- safe to issue spectrum data
  TWI0.SCTRLA |= TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm;
#else
  Serial.println();
#endif

}

/*
  Top frequency w/10 or 20 MHz F_CPU (figures rounded to nearest int)
  Values would be different if 12 or 16 MHz is used.
  SAMPCTRL  SAMPLE RATE  TOP FREQ (Hz)
    0        24038       12019
    1        22321       11161
    2        20833       10417
    3        19531        9766
    4        18382        9191
    5        17361        8681
    6        16447        8224
    7        15625        7813
    8        14881        7440
    9        14205        7102
    10       13587        6793
    11       13021        6510
    12       12500        6250 ** default
    13       12019        6010
    14       11574        5787
    15       11161        5580
    16       10776        5388
    17       10417        5208
    18       10081        5040
    19        9766        4883
    20        9470        4735
    21        9191        4596
    22        8929        4464
    23        8681        4340
    24        8446        4223
    25        8224        4112
    26        8013        4006
    27        7813        3906
    28        7622        3811
    29        7440        3720
    30        7267        3634
    31        7102        3551
*/
#endif
