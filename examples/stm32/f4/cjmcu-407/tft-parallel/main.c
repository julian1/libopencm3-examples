/*
  8-bit parallel.

  in 8 bit parallel.
    is the command still 16 bit.
    which byte is loaded first.



  - general
    https://controllerstech.com/interface-tft-display-with-stm32/

  https://www.displayfuture.com/Display/datasheet/controller/ILI9341.pdf

  8 bit parallel,
  https://github.com/sammyizimmy/ili9341/blob/master/ili9341.c

  simple,
  https://github.com/adafruit/Adafruit_ILI9341

  primitives
    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.cpp

  optimised. 70 forks. but uses spi.
    https://github.com/PaulStoffregen/ILI9341_t3

  So we would have to bit-bash.
    we can use the millisec msleep to begin with.
    a nop loop for micro-second. just write the inner loop - execute it a thousand times and blink led -

  Note - rather than setting gpio individually - would have to left/shift values into place for the bus.
    but would likely be pretty damn fast.
  --
 */


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"
#include "Adafruit_ILI9341.h"

// GPIOE
#define LCD_PORT  GPIOE

#define LCD_RST   GPIO2
#define LCD_CS    GPIO3
#define LCD_RS    GPIO4
#define LCD_WR    GPIO5
#define LCD_RD    GPIO6


#define LCD_DATA_PORT  GPIOD

// LCD

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


// should put control registers on different port - this would allow 16 bit parallel bit bashing
// without hardly any code changes.


// OK. really not sure if we have to pad everything to 16 bit.
// sendCommand16 is a specialization for 16 bit parallel path.
// eg. should only require 16 bits, if its unavoidable because the bus is genuinely 16 bit.

static inline void delay( uint16_t x )
{
  msleep(x);
}



static void send8( uint8_t x )
{
  gpio_clear(LCD_PORT, LCD_WR);         // clear write strobe
  gpio_port_write(LCD_DATA_PORT, x);    // bytes
  delay(1);
  gpio_set(LCD_PORT, LCD_WR);           // write on rising edge
  delay(1);
}


static void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes)
{
  gpio_clear(LCD_PORT, LCD_CS);   // assert chip select

  gpio_clear(LCD_PORT, LCD_RS);   // assert command
  send8(command);

  gpio_set(LCD_PORT, LCD_RS);     // assert data
  for(unsigned i = 0; i < numDataBytes; ++i) {
    send8(dataBytes[ i ]);
  }

  gpio_set(LCD_PORT, LCD_CS);     // deassert chip select
}



// static const uint8_t PROGMEM initcmd[] = {
static uint8_t initcmd[] = {
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


static uint8_t pgm_read_byte(const uint8_t *addr) { return *addr; } 


int main(void)
{
  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOD );


  gpio_mode_setup(LCD_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0xff ); // JA first 8 bits.
  gpio_mode_setup(LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_RST | LCD_CS | LCD_RS | LCD_WR | LCD_RD);


  // reset
  gpio_set(  LCD_PORT, LCD_RST);   
  gpio_clear(LCD_PORT, LCD_RST);   
  msleep(150);
  gpio_set(  LCD_PORT, LCD_RST);   
  msleep(150);


  // ok running this - changes the screen brightness down a bit. or something
  //  
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    // Ok, hi bit determines if needs a delay... horrible
    if (x & 0x80)
      delay(150);
  }


  bool invert = 0;
 	while (1) {

    // gpio_toggle(GPIOD, 1 << 5 );  // blink

    sendCommand(invert ? ILI9341_INVON : ILI9341_INVOFF, 0 , 0 );
    invert = ! invert;

    gpio_toggle(GPIOE, GPIO0);  // toggle led
    msleep(300);
	}

  return 0;
}


