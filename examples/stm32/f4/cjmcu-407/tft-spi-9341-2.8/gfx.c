

//#include "Adafruit_GFX.h"

#include <stdint.h> // uint16_t

#include "Adafruit-GFX-Library/gfxfont.h"
#include "Adafruit-GFX-Library/glcdfont.c"


int x;


inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}


