#if CONFIG_FHT && defined(MEGATINYCORE)

#define FHT_N 128
#define LOG_OUT 1

#include <FHT.h>
//#include "tables.h"

//#define DISABLE_MILLIS // Avail in megaTinyCore Tools menu, but backup here

volatile uint8_t counter = 0; // For filling FHT input buffer

#endif // end CONFIG_FHT


#if 0
#define ANALOG_MUX 4  // AIN# to use (not always same as Arduino analog pin #)
// AIN4 is PA4 on ATtiny817

// PB1/PB0 is SDA/SCL on Xplained board, but depending on core config,
// might be PA1/PA2 for SDA/SCL elsewhere. Analog input has been moved
// to AIN4 (PA4) just in case (previously PA1).




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

void setup() {
  pinMode(7, OUTPUT); // PB4 = LED0 on XPlained board
  digitalWrite(7, HIGH); // HIGH = LED off on XPlained board

#ifdef DISABLE_MILLIS
#if defined(MILLIS_USE_TIMERA0)
  TCA0.SPLIT.INTCTRL &= ~TCA_SPLIT_HUNF_bm;
#elif defined(MILLIS_USE_TIMERA1)
  TCA1.SPLIT.INTCTRL &= ~TCA_SPLIT_HUNF_bm;
#elif defined(MILLIS_USE_TIMERB0)
  TCB0.INTCTRL &= ~TCB_CAPT_bm;
#elif defined(MILLIS_USE_TIMERB1)
  TCB1.INTCTRL &= ~TCB_CAPT_bm;
#elif defined(MILLIS_USE_TIMERD0)
  TCD0.INTCTRL &= ~TCD_OVF_bm;
#endif
#endif // end DISABLE_MILLIS

  // ADC is configured for free-run mode with result-ready interrupt. 10-bit
  // w/4X accumulation for 12-bit result (0-4092, NOT 4095, because it's the
  // sum of four 10-bit values, not "true" 12-bit ADC reading).
  // 1.25 MHz / 4X samples / 25 ADC cycles/sample -> 12500 Hz sample rate.
  // Highest frequency is 1/2 sampling rate, or 6250 Hz (just under G8 at
  // 6272 Hz). That’s a default that looks nice, but there's some adjustability
  // if needed, with the following top frequency range:
  // 1.25 MHz / 4X / (13+0)  = 24038 sample rate = 12019 peak freq
  // 1.25 MHz / 4X / (13+31) = 7102 sample rate = 3551 peak freq
  // 32 possible values, see notes at bottom for corresponding frequencies.
  // This also affects the "frame rate" or how frequently we can record
  // 128 samples and perform the FHT.
  // In very rough, colloquial and layman-ish terms, it would be reasonable
  // enough to call this "3.5 to 12 KHz adjustable peak frequency," there's
  // going to be some oscillator variance and so forth anyway.
  // ALSO: depending on what mic is used, might want to change AREF to
  // another source. Right now it's the default VDD.

  ADC0.CTRLA = ADC_FREERUN_bm | ADC_ENABLE_bm; // 10-bit, free-run, enable ADC
  ADC0.CTRLB = ADC_SAMPNUM_ACC4_gc;   // Accumulate 4X (0-4092 (sic.) result)
  ADC0.CTRLC = ADC_SAMPCAP_bm |       // Reduced capacitance for >1V AREF
               ADC_REFSEL_VDDREF_gc | // VDD as AREF
#if F_CPU > 12000000
               ADC_PRESC_DIV16_gc;    // 16:1 timer prescale (20->1.25 MHz)
#else
               ADC_PRESC_DIV8_gc;     // 8:1 timer prescale (10->1.25 MHz)
#endif
  ADC0.CTRLD = 0;           // No init or sample delay
  ADC0.MUXPOS = ANALOG_MUX; // Select pin for analog input
  ADC0.SAMPCTRL = 12;       // Add to usu. 13 ADC cycles for 25 cycles/sample
  ADC0.INTCTRL |= ADC_RESRDY_bm; // Enable result-ready interrupt
  ADC0.COMMAND |= ADC_STCONV_bm; // Start free-run conversion
}

ISR(ADC0_RESRDY_vect) { // ADC conversion complete
  // Convert 12-bit ADC reading to signed value (+/-2K) and scale to 16-bit
  // space (scaling up isn't strictly required but the FHT results look much
  // cleaner). 2046 (not 2048) is intentional, see ADC notes above, don't "fix."
  fht_input[counter] = (ADC0.RES - 2046) * 4;
  if (++counter >= FHT_N) {         // FHT input buffer full?
    ADC0.INTCTRL &= ~ADC_RESRDY_bm; // Disable result-ready interrupt
  }
  // Interrupt flag is cleared automatically when reading ADC0.RES
}

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
