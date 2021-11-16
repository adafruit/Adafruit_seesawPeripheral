static uint8_t noise[64] = {
  // These figures came from recording "silence" but might be
  // thrown off by ambient noise during recording (computer fans, etc.)
  74, 65, 29, 29, 19, 25, 22, 23, 12, 21, 19, 20, 16, 18, 18, 18,
  4, 14, 14, 14, 12, 15, 14, 14, 8, 13, 12, 13, 11, 12, 11, 11,
  4, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8,
  3, 7, 7, 4, 4, 5, 5, 6, 4, 5, 5, 5, 5, 5, 5, 4
  // Also probably a function of the mic/amp circuit being used.
  // Might want to re-do this with final hardware.
};

// Same deal, these came from recording frequency sweeps at high volume,
// might be limited by mic, etc. and need a re-do w/final hardware.
static uint8_t peak[64] = {
  180, 188, 189, 188, 179, 185, 178, 187, 187, 185, 186, 189, 185, 188, 188, 180,
  186, 187, 188, 188, 188, 188, 188, 187, 187, 187, 186, 177, 185, 186, 185, 183,
  182, 181, 182, 171, 162, 170, 180, 180, 178, 164, 175, 172, 161, 153, 150, 159,
  159, 163, 159, 155, 151, 152, 152, 153, 164, 174, 178, 171, 176, 178, 178, 171 };
static uint16_t scale[64] = {
  364, 348, 346, 348, 366, 354, 368, 350, 350, 354, 352, 346, 354, 348, 348, 364,
  352, 350, 348, 348, 348, 348, 348, 350, 350, 350, 352, 370, 354, 352, 354, 358,
  360, 362, 360, 383, 404, 385, 364, 364, 368, 399, 374, 381, 407, 428, 436, 412,
  412, 402, 412, 422, 434, 431, 431, 428, 399, 376, 368, 383, 372, 368, 368, 383 };
