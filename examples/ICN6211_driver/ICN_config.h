const uint8_t PROGMEM ICN6211_TL040HD[] = {
  0x00, 0xc1,
  0x01, 0x62,
  0x02, 0x11,
  0x03, 0xff,
  0x08, 0x00,
  0x09, 0x10,
  0x10, 0x30,
  0x11, 0x88,
  0x14, 0x83,
  0x20, 0xd0,
  0x21, 0xd0,
  0x22, 0x22,
  0x23, 0x2e,
  0x24, 0x2c,
  0x25, 0x02,
  0x26, 0x00,
  0x27, 0x10,
  0x28, 0x02,
  0x29, 0x12,
  0x2a, 0x01,
  0x31, 0x78,
  0x32, 0x04,
  0x33, 0xff,
  0x34, 0x80,
  0x35, 0x08,
  0x36, 0x2e,
  0x51, 0x20,
  0x56, 0x92,
  0x63, 0xff,
  0x64, 0x00,
  0x65, 0x00,
  0x66, 0x00,
  0x67, 0x00,
  0x68, 0x00,
  0x69, 0x03,
  0x6a, 0x00,
  0x6b, 0x31,
  0x7a, 0x3e,
  0x7b, 0xff,
  0x7c, 0xf1,
  0x7d, 0x01,
  0x80, 0x00,
  0x81, 0x00,
  0x84, 0x01,
  0x85, 0x00,
  0x86, 0x28,
  0x87, 0x00,
  0x90, 0x05,
  0x91, 0x0a,
  0x92, 0x18,
  0x94, 0x0d,
  0x95, 0x04,
  0x96, 0x64,
  0x97, 0x00,
  0x99, 0x05,
  0x9a, 0x96,
  0xb5, 0xa0,
  0xb6, 0x20,
  0xFF, 0xFF
};


const uint8_t PROGMEM ICN6211_TL060HDV07[]= {
  0x00, 0xc1,
  0x01, 0x62,
  0x02, 0x11,
  0x03, 0xff,
  0x08, 0x00,
  0x09, 0x10,
  0x10, 0x30,
  0x11, 0x88,
  0x14, 0x43,
  0x20, 0xd0,
  0x21, 0xa0,
  0x22, 0x52,
  0x23, 0x2e,
  0x24, 0x2c,
  0x25, 0x02,
  0x26, 0x00,
  0x27, 0x10,
  0x28, 0x02,
  0x29, 0x12,
  0x2a, 0x29,
  0x31, 0x78,
  0x32, 0x04,
  0x33, 0xff,
  0x34, 0x80,
  0x35, 0x08,
  0x36, 0x2e,
  0x51, 0x20,
  0x56, 0x92,
  0x63, 0xff,
  0x64, 0x00,
  0x65, 0x00,
  0x66, 0x00,
  0x67, 0x00,
  0x68, 0x00,
  0x69, 0x03,
  0x6a, 0x00,
  0x6b, 0x31,
  0x7a, 0x3e,
  0x7b, 0xff,
  0x7c, 0xf1,
  0x7d, 0x01,
  0x80, 0x00,
  0x81, 0x00,
  0x84, 0x01,
  0x85, 0x00,
  0x86, 0x28,
  0x87, 0x00,
  0x90, 0x05,
  0x91, 0x0a,
  0x92, 0x18,
  0x94, 0x0d,
  0x95, 0x04,
  0x96, 0x64,
  0x97, 0x00,
  0x99, 0x05,
  0x9a, 0x96,
  0xb5, 0xa0,
  0xb6, 0x20,
  0xFF, 0xFF
};

/* 
dpi_output_format=0x6f016
hdmi_timings=480 0 100 10 10 800 1 10 2 20 0 0 0 60 0 32000000 6
hpol = 0
hfp = 100
hsync = 10
hbp = 10
vlines = 800
vpol = 1
vfp = 10
vsync = 2
vbp = 20
no offsets
de pulses low (mostly high)
hsync & vsync pulse high (mostly low)
pclk ~32MHz, 60Hz refresh
 */
const uint8_t PROGMEM ICN6211_HM040HQ40R[]= {
0x00, 0xc1,
0x01, 0x62,
0x02, 0x11,
0x03, 0xff,
0x08, 0x00,
0x09, 0x10,
0x10, 0x30,
0x11, 0x88,
0x14, 0x43,
0x20, 0xe0,
0x21, 0x20,
0x22, 0x31,
0x23, 0x64,
0x24, 0x0a,
0x25, 0x0a,
0x26, 0x00,
0x27, 0x0a,
0x28, 0x02,
0x29, 0x14,
0x2a, 0x4f,
0x31, 0x78,
0x32, 0x04,
0x33, 0xff,
0x34, 0x80,
0x35, 0x08,
0x36, 0x64,
0x51, 0x20,
0x56, 0x92,
0x63, 0xff,
0x64, 0x00,
0x65, 0x00,
0x66, 0x00,
0x67, 0x00,
0x68, 0x00,
0x69, 0x06,
0x6a, 0x00,
0x6b, 0x71,
0x7a, 0x3e,
0x7b, 0xff,
0x7c, 0xf1,
0x7d, 0x01,
0x80, 0x00,
0x81, 0x00,
0x84, 0x01,
0x85, 0x00,
0x86, 0x28,
0x87, 0x00,
0x90, 0x05,
0x91, 0x0a,
0x92, 0x18,
0x94, 0x0d,
0x95, 0x04,
0x96, 0x64,
0x97, 0x00,
0x99, 0x05,
0x9a, 0x96,
0xb5, 0xa0,
0xb6, 0x20,
  0xFF, 0xFF
};
