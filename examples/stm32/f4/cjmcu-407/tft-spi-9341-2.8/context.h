
#include <stdint.h>


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


typedef struct Context
{
    // could almost include opaque spi structure here, if needed

    uint8_t rotation  ;
    uint16_t width;
    uint16_t height;



} Context;


void delay( uint16_t x ); 

void initialize(Context *ctx);

void ILI9341_setRotation(Context *ctx, uint8_t m) ;

void ILI9341_SetAddressWindow(Context *ctx, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

void ILI9341_DrawRectangle(Context *ctx, uint16_t x, uint16_t y, uint16_t x_off, uint16_t y_off, uint16_t color);



