

#include <stdint.h> // uint16_t etc

#include "Adafruit-GFX-Library/gfxfont.h"
#include "Adafruit-GFX-Library/glcdfont.c"


#include "lcd_spi.h"
#include "Adafruit_ILI9341.h"

#include "context.h"



/*
inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}
*/

////////////////////////////////

// clang-format off
static const uint8_t initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};
// clang-format on



static uint8_t pgm_read_byte(const uint8_t *addr) {
  return *addr;
}



void initialize( void)
{


    // not sure if this is the best place
  assert_cs();
  delay(1);


  // hardware reset - review
  // gpio_set(  LCD_CTL_PORT, LCD_CTL_RST);    // high
  deassert_rst();
  delay(150);
  assert_rst();
  // gpio_clear(LCD_CTL_PORT, LCD_CTL_RST);   // low
  delay(150);
  // gpio_set(  LCD_CTL_PORT, LCD_CTL_RST);   // high
  deassert_rst();
  delay(150);


  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    // Ok, hi bit determines if needs a delay... horrible
    // single argument and delay...
    if (x & 0x80)
      delay(150);
  }
}






#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

// OK. we are going to need a struct to handle this...

/*
uint8_t rotation  ;
uint16_t _width;
uint16_t _height;
*/

// cvoid ILI9341_setRotation(uint8_t m) {
void ILI9341_setRotation(Context *ctx, uint8_t m)
{

  ctx->rotation = m % 4; // can't be higher than 3

  switch (ctx->rotation) {
    case 0:
      m = (MADCTL_MX | MADCTL_BGR);
      ctx->width = ILI9341_TFTWIDTH;
      ctx->height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      m = (MADCTL_MV | MADCTL_BGR);
      ctx->width = ILI9341_TFTHEIGHT;
      ctx->height = ILI9341_TFTWIDTH;
      break;
    case 2:
      m = (MADCTL_MY | MADCTL_BGR);
      ctx->width = ILI9341_TFTWIDTH;
      ctx->height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      ctx->width = ILI9341_TFTHEIGHT;
      ctx->height = ILI9341_TFTWIDTH;
      break;
  }

  sendCommand(ILI9341_MADCTL, &m, 1);
}




// we need the low level functions.

void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  {
      uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
      // ILI9341_WriteData(data, sizeof(data));
      sendCommand(ILI9341_CASET, data, sizeof(data) ); // 2A
  }

  // ILI9341_WriteCommand(0x2B); // RASET
  {
      uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
      sendCommand(ILI9341_PASET, data, sizeof(data) ); // 2B
      // ILI9341_WriteData(data, sizeof(data));
  }
}



void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t x_off, uint16_t y_off, uint16_t color)
{

  // TODO clamp inputs..

  ILI9341_SetAddressWindow(x, y, x + x_off - 1, y + y_off - 1);

  // send command
  sendCommand0(ILI9341_RAMWR ); // 2C ram write
  //delay(1);

  set_data();
  // delay(1);

  // 16 bit color
  int i;
  for( i = 0; i < x_off * y_off ; ++i) {
    send8( color >> 8 );
    send8( color & 0xFF );
  }
}



