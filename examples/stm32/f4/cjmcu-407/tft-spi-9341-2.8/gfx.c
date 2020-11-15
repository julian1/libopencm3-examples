
#include "context.h"
#include "gfx.h"


#define UNUSED(x) (void)(x)

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

#if 0
void Adafruit_SPITFT::drawPixel(int16_t x, int16_t y, uint16_t color) {
  // Clip first...
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
    // THEN set up transaction (if needed) and draw...
    startWrite();
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
    endWrite();
  }
}
#endif

static void drawPixel(Context *ctx, int16_t x, int16_t y, uint16_t color) {
  // Clip first...
  if ((x >= 0) && (x < ctx->width) && (y >= 0) && (y < ctx->height)) {
    // THEN set up transaction (if needed) and draw...


    //  can improve later
    ILI9341_DrawRectangle(ctx, x, y, 1 , 1, color);

  }
}


void writePixel(Context *ctx, int16_t x, int16_t y, uint16_t color) {
  drawPixel(ctx, x, y, color);
}

#include <stdlib.h> // abs


#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif



void writeLine(Context *ctx, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
#if defined(ESP8266)
  yield();
#endif
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      writePixel(ctx, y0, x0, color);
    } else {
      writePixel(ctx, x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}





void drawCircle(Context *ctx, int16_t x0, int16_t y0, int16_t r, uint16_t color) 
{
#if defined(ESP8266)
  yield();
#endif
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  startWrite(ctx);
  writePixel(ctx, x0, y0 + r, color);
  writePixel(ctx, x0, y0 - r, color);
  writePixel(ctx, x0 + r, y0, color);
  writePixel(ctx, x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    writePixel(ctx, x0 + x, y0 + y, color);
    writePixel(ctx, x0 - x, y0 + y, color);
    writePixel(ctx, x0 + x, y0 - y, color);
    writePixel(ctx, x0 - x, y0 - y, color);
    writePixel(ctx, x0 + y, y0 + x, color);
    writePixel(ctx, x0 - y, y0 + x, color);
    writePixel(ctx, x0 + y, y0 - x, color);
    writePixel(ctx, x0 - y, y0 - x, color);
  }
  endWrite(ctx);
}



