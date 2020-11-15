
#include "context.h"
#include "gfx.h"

#if 0
void Adafruit_GFX::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color) {
  startWrite();
  for (int16_t i = x; i < x + w; i++) {
    writeFastVLine(i, y, h, color);
  }
  endWrite();
}
#endif




void fillRect(Context *ctx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{

  ILI9341_DrawRectangle(ctx, x, y, w, h, color);
#if 0
  startWrite();
  for (int16_t i = x; i < x + w; i++) {
    writeFastVLine(i, y, h, color);
  }
  endWrite();
#endif
}

void fillScreen(Context *ctx, uint16_t color) 
{
  fillRect(ctx, 0, 0, ctx->width, ctx->height, color);
}

/*
    appears to use a buffer????
void writePixel(Context *ctx, int16_t x, int16_t y, uint16_t color) 
{
  drawPixel(x, y, color);
}
*/

void writeFillRect(Context *ctx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // same...
  fillRect(ctx, x, y, w, h, color);
}


void writeFastHLine(Context *ctx, int16_t x, int16_t y, int16_t w, uint16_t color) 
{
  // Overwrite in subclasses if startWrite is defined!
  // Example: writeLine(x, y, x+w-1, y, color);
  // or writeFillRect(x, y, w, 1, color);
  //drawFastHLine(x, y, w, color);

  writeFillRect(ctx, x, y, w, 1, color);

}


