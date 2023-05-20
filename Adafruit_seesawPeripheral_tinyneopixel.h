void tinyNeoPixel_show(uint8_t pin, uint16_t numBytes, uint8_t *pixels) {

#ifdef CONFIG_NEOPIXEL_ACT_LED
  pinMode(CONFIG_NEOPIXEL_ACT_LED, OUTPUT);
  digitalWrite(CONFIG_NEOPIXEL_ACT_LED, LOW);
#endif

  // AVRxt MCUs --  tinyAVR 0/1/2, megaAVR 0, AVR Dx ----------------------
  // with extended maximum speeds to support vigorously overclocked
  // Dx-series parts. This is by no means intended to imply that they will
  // run at those speeds, only that - if they do - you can control WS2812s
  // with them.

  volatile uint8_t *port    = portOutputRegister(digitalPinToPort(pin));
  uint8_t pinMask = digitalPinToBitMask(pin);

  volatile uint16_t
    i   = numBytes; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low

  // Hand-tuned assembly code issues data to the LED drivers at a specific
  // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
  // for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
  // datastream timing for the LED drivers allows a little wiggle room each
  // way (listed in the datasheets), so the conditions for compiling each
  // case are set up for a range of frequencies rather than just the exact
  // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
  // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
  // on the datasheet figures and have not been extensively tested outside
  // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
  // close to the extremes (or possibly they could be pushed further).
  // Keep in mind only one CPU speed case actually gets compiled; the
  // resulting program isn't as massive as it might look from source here.

  // 8 MHz(ish) AVRxt ---------------------------------------------------------
  #if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

    volatile uint8_t n1, n2 = 0;  // First, next bits out

    // We need to be able to write to the port register in one clock
    // to meet timing constraints here.

    // 10 instruction clocks per bit: HHxxxxxLLL
    // OUT instructions:              ^ ^    ^   (T=0,2,7)

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    n1 = lo;
    if (b & 0x80) n1 = hi;

    // Dirty trick: RJMPs proceeding to the next instruction are used
    // to delay two clock cycles in one instruction word (rather than
    // using two NOPs).  This was necessary in order to squeeze the
    // loop down to exactly 64 words -- the maximum possible for a
    // relative branch.

    asm volatile(
     "headD:"                   "\n\t" // Clk  Pseudocode
      // Bit 7:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
      "st   %a[port], %[n1]"    "\n\t" // 1    PORT = n1
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 6"        "\n\t" // 1-2  if (b & 0x40)
       "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 6:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
      "st   %a[port], %[n2]"    "\n\t" // 1    PORT = n2
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 5"        "\n\t" // 1-2  if (b & 0x20)
       "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 5:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
      "st   %a[port], %[n1]"    "\n\t" // 1    PORT = n1
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 4"        "\n\t" // 1-2  if (b & 0x10)
       "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 4:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
      "st   %a[port], %[n2]"    "\n\t" // 1    PORT = n2
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 3"        "\n\t" // 1-2  if (b & 0x08)
       "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 3:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
      "st   %a[port], %[n1]"    "\n\t" // 1    PORT = n1
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 2"        "\n\t" // 1-2  if (b & 0x04)
       "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 2:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
      "st   %a[port], %[n2]"    "\n\t" // 1    PORT = n2
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 1"        "\n\t" // 1-2  if (b & 0x02)
       "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "rjmp .+0"                "\n\t" // 2    nop nop
      // Bit 1:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
      "st   %a[port], %[n1]"    "\n\t" // 1    PORT = n1
      "rjmp .+0"                "\n\t" // 2    nop nop
      "sbrc %[byte] , 0"        "\n\t" // 1-2  if (b & 0x01)
       "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "sbiw %[count], 1"        "\n\t" // 2    i-- (don't act on Z flag yet)
      // Bit 0:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi
      "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
      "st   %a[port], %[n2]"    "\n\t" // 1    PORT = n2
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if (b & 0x80)
       "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo
      "brne headD"              "\n"   // 2    while(i) (Z flag set above)
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [n1]    "+r" (n1),
      [n2]    "+r" (n2),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

  #elif (F_CPU >= 9500000UL) && (F_CPU <= 11100000UL)
    /*
    volatile uint8_t n1, n2 = 0;  // First, next bits out

    */
    // 14 instruction clocks per bit: HHHHxxxxLLLLL
    // ST instructions:               ^   ^   ^   (T=0,4,7)
    volatile uint8_t next;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    if (b & 0x80) {
      next = hi;
    }

    // Don't "optimize" the OUT calls into the bitTime subroutine;
    // we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
    asm volatile(
     "headD:"                   "\n\t" //        (T =  0)
      "st   %a[port], %[hi]"    "\n\t" //        (T =  1)
      "rcall bitTimeD"          "\n\t" // Bit 7  (T = 14)
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 6
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 5
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 4
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 3
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 2
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 1
      // Bit 0:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi    (T =  1)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++   (T =  5)
      "st   %a[port], %[next]"  "\n\t" // 1    PORT = next  (T =  6)
      "rjmp .+0"                "\n\t" // 2    nop nop      (T =  3)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo    (T =  7)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if (b & 0x80) (T =  8)
       "mov %[next] , %[hi]"    "\n\t" // 0-1    next = hi  (T =  9)
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo    (T = 10)
      "sbiw %[count], 1"        "\n\t" // 2    i--          (T = 12)
      "brne headD"              "\n\t" // 2    if (i != 0) -> (next byte)
       "rjmp doneD"             "\n\t"
      "bitTimeD:"               "\n\t" //      nop nop nop     (T =  4)
       "st   %a[port], %[next]" "\n\t" // 1    PORT = next     (T =  5)
       "mov  %[next], %[lo]"    "\n\t" // 1    next = lo       (T =  6)
       "rol  %[byte]"           "\n\t" // 1    b <<= 1         (T =  7)
       "sbrc %[byte], 7"        "\n\t" // 1-2  if (b & 0x80)    (T =  8)
        "mov %[next], %[hi]"    "\n\t" // 0-1   next = hi      (T =  9)
       "st   %a[port], %[lo]"   "\n\t" // 1    PORT = lo       (T = 10)
       "ret"                    "\n\t" // 4    nop nop nop nop (T = 14)
       "doneD:"                 "\n"
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));



// 12 MHz(ish) AVRxt --------------------------------------------------------
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)

    // In the 12 MHz case, an optimized 800 KHz datastream (no dead time
    // between bytes) requires a PORT-specific loop similar to the 8 MHz
    // code (but a little more relaxed in this case).

    // 15 instruction clocks per bit: HHHHxxxxxxLLLLL
    // OUT instructions:              ^   ^     ^     (T=0,4,10)

    volatile uint8_t next;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    if (b & 0x80) {
      next = hi;
    }

      // Don't "optimize" the OUT calls into the bitTime subroutine;
      // we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
    asm volatile(
     "headD:"                   "\n\t" //        (T =  0)
      "st   %a[port], %[hi]"    "\n\t" //        (T =  1)
      "rcall bitTimeD"          "\n\t" // Bit 7  (T = 15)
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 6
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 5
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 4
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 3
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 2
      "st   %a[port], %[hi]"    "\n\t"
      "rcall bitTimeD"          "\n\t" // Bit 1
      // Bit 0:
      "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi    (T =  1)
      "rjmp .+0"                "\n\t" // 2    nop nop      (T =  3)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++   (T =  5)
      "st   %a[port], %[next]"  "\n\t" // 1    PORT = next  (T =  6)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo    (T =  7)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if (b & 0x80) (T =  8)
       "mov %[next] , %[hi]"    "\n\t" // 0-1    next = hi  (T =  9)
      "nop"                     "\n\t" // 1                 (T = 10)
      "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo    (T = 11)
      "sbiw %[count], 1"        "\n\t" // 2    i--          (T = 13)
      "brne headD"              "\n\t" // 2    if (i != 0) -> (next byte)
       "rjmp doneD"             "\n\t"
      "bitTimeD:"               "\n\t" //      nop nop nop     (T =  4)
       "st   %a[port], %[next]" "\n\t" // 1    PORT = next     (T =  5)
       "mov  %[next], %[lo]"    "\n\t" // 1    next = lo       (T =  6)
       "rol  %[byte]"           "\n\t" // 1    b <<= 1         (T =  7)
       "sbrc %[byte], 7"        "\n\t" // 1-2  if (b & 0x80)    (T =  8)
        "mov %[next], %[hi]"    "\n\t" // 0-1   next = hi      (T =  9)
       "nop"                    "\n\t" // 1                    (T = 10)
       "st   %a[port], %[lo]"   "\n\t" // 1    PORT = lo       (T = 11)
       "ret"                    "\n\t" // 4    nop nop nop nop (T = 15)
       "doneD:"                 "\n"
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));


// 16 MHz(ish) AVRxt ------------------------------------------------------
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

    // WS2811 and WS2812 have different hi/lo duty cycles; this is
    // similar but NOT an exact copy of the prior 400-on-8 code.

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^    ^       ^       (T=0,5,13)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "nop"                      "\n\t" // 1    nop           (T =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T =  6)
      "nop"                      "\n\t" // 1    nop           (T =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
      "nop"                      "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 14)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out) (T=20)
     "nextbyte20:"               "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 14)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
       "brne head20"             "\n"   // 2    if (i != 0) -> (next byte) (T=20)
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

// 20 MHz(ish) AVRxt ------------------------------------------------------
#elif (F_CPU >= 19000000UL) && (F_CPU <= 22000000L)


    // 25 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,7,15)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "nop"                      "\n\t" // 1    nop           (T =  5)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T =  7)
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T =  8)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  9)
      "breq nextbyte20"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 11)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 13)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 15)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 16)
      "nop"                      "\n\t" // 1    nop           (T = 17)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 19)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 21)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 23)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (T = 11)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 12)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 14)
      "nop"                      "\n\t" // 1    nop           (T = 15)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 16)
      "nop"                      "\n\t" // 1    nop           (T = 17)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 19)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 21)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 23)
       "brne head20"             "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

// 24 (22~26) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU >= 22000000UL) && (F_CPU <= 26000000L)


    // 30 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,9,18)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;


    asm volatile(
     "head24:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
      "mov  %[next], %[hi]"      "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "nop"                      "\n\t" // 1    nop           (T =  5)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T =  7)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T =  9)
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 10)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 11)
      "nop"                      "\n\t" // 1    nop           (T = 12)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 14)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 16)
      "breq nextbyte24"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 18)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 19)
      "rcall seconddelay24"      "\n\t" // 2+4+3=9            (T = 28)
      "rjmp head24"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay24:"            "\n\t" //
      "nop"                      "\n\t" // 1
      "rjmp .+0"                 "\n\t" // 2
      "ret"                      "\n\t" // 4
     "nextbyte24:"               "\n\t" // last bit of a byte (T = 18)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 19)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 20)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 22)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 24)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 26)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 28)
      "brne head24"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));




// 28 (26~30) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU >= 26000000UL) && (F_CPU <= 30000000L)


    // 35 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,10,21)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head28:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "rcall zerothdelay32"      "\n\t" // 2+4=6
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 11)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 12)
      "rcall firstdelay28"       "\n\t" // 2+4 = 7            (T = 19)
      "breq nextbyte28"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 21)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 22)
      "rcall seconddelay28"      "\n\t" // 2+4+1+4=11         (T = 33)
      "rjmp head28"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay28:"            "\n\t" //
      "rjmp .+0"                 "\n\t" // 2
      "rjmp .+0"                 "\n\t" // 2
     "firstdelay28:"             "\n\t" // first delay
      "nop"                      "\n\t" // 1    nop
     "thirddelay28:"             "\n\t" // third delay
     "zerothdelay28:"            "\n\t"
      "ret"                      "\n\t" // 4
     "nextbyte28:"               "\n\t" // last bit of a byte (T = 21)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 22)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 23)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 25)
      "rcall thirddelay28"       "\n\t" // 2+4 = 6            (T = 31)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 33)
      "brne head28"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));


// 32 (30~34) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU > 30000000UL) && (F_CPU <= 34000000L)


    // 40 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,11,24)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head32:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "rcall zerothdelay32"      "\n\t" // 2+4+1=7
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 12)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 13)
      "rcall firstdelay32"       "\n\t" // 2+4+1+2 = 9        (T = 22)
      "breq nextbyte32"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 24)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 25)
      "rcall seconddelay32"      "\n\t" // 2+4+3+2+3=13       (T = 38)
      "rjmp head32"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay32:"            "\n\t" // second delay 13 cycles
      "rjmp .+0"                 "\n\t" // 2
      "rjmp .+0"                 "\n\t" // 2
     "firstdelay32:"             "\n\t" // first delay 9 cycles
      "nop"                      "\n\t" // 1    nop
     "thirddelay32:"             "\n\t" // third delay 8 cycles
      "nop"                      "\n\t" // 1    nop
     "zerothdelay32:"            "\n\t" // zeroth delay 7 cycles
      "nop"                      "\n\t" // 1    nop
      "ret"                      "\n\t" // 4
     "nextbyte32:"               "\n\t" // last bit of a byte (T = 24)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 25)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 26)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 28)
      "rcall thirddelay32"       "\n\t" // 2+4+1+1 = 8        (T = 36)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 38)
      "brne head32"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

// 36 (34~38) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU > 3400000UL) && (F_CPU <= 38000000L)


    // 45 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,12,27)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head36:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "rcall zerothdelay36"      "\n\t" // 2+4+2=8
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 13)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 14)
      "rcall firstdelay36"       "\n\t" // 2+4+3 = 11         (T = 25)
      "breq nextbyte36"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 27)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 28)
      "rcall seconddelay36"      "\n\t" // 2+4+3+2+2=15       (T = 43)
      "rjmp head36"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay36:"            "\n\t" // second delay 15 cycles
      "rjmp .+0"                 "\n\t" // 2
      "rjmp .+0"                 "\n\t" // 2
     "firstdelay36:"             "\n\t" // first delay 11 cycles
      "nop"                      "\n\t" // 1    nop
     "thirddelay36:"             "\n\t" // third delay 10 cycles
      "rjmp .+0"                 "\n\t" // 2    nop nop
     "zerothdelay36:"            "\n\t" // zeroth delay 8 cycles
      "rjmp .+0"                 "\n\t" // 2    nop nop
      "ret"                      "\n\t" // 4
     "nextbyte36:"               "\n\t" // last bit of a byte (T = 27)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 28)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 29)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 31)
      "rcall thirddelay36"       "\n\t" // 2+4 = 10           (T = 41)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 43)
      "brne head36"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));


// 40 (38-44) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU > 3800000UL) && (F_CPU <= 44000000L)


    // 50 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,14,30)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head40:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "rcall zerothdelay40"      "\n\t" // 2+4+4=10
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 15)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 16)
      "rcall firstdelay40"       "\n\t" // 2+4+4+2 = 12         (T = 28)
      "breq nextbyte40"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 30)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 31)
      "rcall seconddelay40"      "\n\t" // 2+4+3+2+3=17       (T = 48)
      "rjmp head40"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay40:"            "\n\t" // second delay 17 cycles
      "nop"                      "\n\t" // 1    nop
      "rjmp .+0"                 "\n\t" // 2
      "rjmp .+0"                 "\n\t" // 2
     "thirddelay40:"             "\n\t" // third delay 12 cycles
     "firstdelay40:"             "\n\t" // first delay 12 cycles
      "rjmp .+0"                 "\n\t" // 2    nop nop
     "zerothdelay40:"            "\n\t" // zeroth delay 10 cycles
      "rjmp .+0"                 "\n\t" // 2    nop nop
      "rjmp .+0"                 "\n\t" // 2    nop nop
      "ret"                      "\n\t" // 4
     "nextbyte40:"               "\n\t" // last bit of a byte (T = 30)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 31)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 32)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 34)
      "rcall thirddelay40"       "\n\t" // 2+4+4+2 = 12       (T = 46)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 48)
      "brne head40"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

// 48 (44-50) MHz AVRxt  ------------------------------------------------------
#elif (F_CPU > 4400000UL) && (F_CPU <= 50000000L)


    // 60 inst. clocks per bit: HHHHHHHxxxxxxxxLLLLLLLLLL
    // ST instructions:         ^      ^       ^       (T=0,16,35)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;
    asm volatile(
     "head48:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if (b & 128)
      "mov  %[next], %[hi]"      "\n\t" // 0-1   next = hi    (T =  3)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  4)
      "rcall zerothdelay48"      "\n\t" // 2+4=13
      "st   %a[port],  %[next]"  "\n\t" // 1    PORT = next   (T = 17)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T = 18)
      "rcall firstdelay48"       "\n\t" // 2+4+3 = 15         (T = 33)
      "breq nextbyte48"          "\n\t" // 1-2  if (bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 35)
      "st   %a[port],  %[lo]"    "\n\t" // 1    PORT = lo     (T = 36)
      "rcall seconddelay48"      "\n\t" // 2+4+3+2+3=22       (T = 58)
      "rjmp head48"              "\n\t" // 2    -> head20 (next bit out)
     "seconddelay48:"            "\n\t" // second delay 22 cycles
      "rjmp .+0"                 "\n\t" // 2
      "rjmp .+0"                 "\n\t" // 2
      "nop"                      "\n\t" // 1    nop
     "thirddelay48:"             "\n\t" // third delay 17 cycles
      "rjmp .+0"                 "\n\t" // 2
     "firstdelay48:"             "\n\t" // first delay 15 cycles
      "rjmp .+0"                 "\n\t" // 2    nop nop
     "zerothdelay48:"            "\n\t" // zeroth delay 13 cycles
      "nop"                      "\n\t" // 1    nop
      "rcall emptydelay48"       "\n\t" // 2+4
      "ret"                      "\n\t" // 4
     "emptydelay48:"             "\n\t" // immediately returns: 2+4 = 6 cycles, for 2 words!
      "ret"                      "\n\t" // 4
     "nextbyte48:"               "\n\t" // last bit of a byte (T = 35)
      "st   %a[port], %[lo]"     "\n\t" // 1    PORT = lo     (T = 36)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 37)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 39)
      "rcall thirddelay48"       "\n\t" // 2+4 = 17           (T = 56)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 58)
      "brne head48"              "\n"   // 2    if (i != 0) -> (next byte)  ()
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

#else
  #error "CPU SPEED NOT SUPPORTED"
#endif


#ifdef CONFIG_NEOPIXEL_ACT_LED
  pinMode(CONFIG_NEOPIXEL_ACT_LED, INPUT);
#endif
}
