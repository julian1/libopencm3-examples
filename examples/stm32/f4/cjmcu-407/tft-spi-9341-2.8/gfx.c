

//#include "Adafruit_GFX.h"

#include <stdint.h> // uint16_t

#include "./Adafruit-GFX-Library/gfxfont.h"

#include "Adafruit-GFX-Library/glcdfont.c"


int x;

#if 0

Adafruit_GFX::Adafruit_GFX(int16_t w, int16_t h) : WIDTH(w), HEIGHT(h) {
  _width = WIDTH;
  _height = HEIGHT;
  rotation = 0;
  cursor_y = cursor_x = 0;
  textsize_x = textsize_y = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap = true;
  _cp437 = false;
  gfxFont = NULL;
}
#endif



inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}


