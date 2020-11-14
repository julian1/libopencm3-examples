/*
  spi 9341  2.8 with touch

  just connecting led/BL to +3.3V Vcc and backlight turns on.
    probably a digital ctrl.  but maybe drawing current.
    so control with gpio later.

  looks like a couple of fets on the back of pcb. but maybe 5V power supplies.
  no tranceivers


  ------------
  - gfx library seems to be a library using a canvas rather than direct draw.
      eg. may be buffering change.

  - general
    https://controllerstech.com/interface-tft-display-with-stm32/

  https://www.displayfuture.com/Display/datasheet/controller/ILI9341.pdf

  8 bit parallel, has init sequence.
  https://github.com/sammyizimmy/ili9341/blob/master/ili9341.c

  simple,
  https://github.com/adafruit/Adafruit_ILI9341

  primitives
    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.cpp

  optimised. 70 forks. but uses spi.
    https://github.com/PaulStoffregen/ILI9341_t3
  --
*/


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"
#include "Adafruit_ILI9341.h"






/*
  spi 1 AF5
    MOSI == PA7 == GPIO7    DAC SDI pin 4
    MISO == PA6 == GPIO6    DAC SDO pin 5
*/
// change name TFT_SPI_CS, TFT_SPI_CLK etc.
#define TFT_SPI       SPI1
#define TFT_SPI_PORT  GPIOA
#define TFT_SPI_AF      // should define.

#define TFT_CS        GPIO4
#define TFT_CLK       GPIO5
#define TFT_MOSI      GPIO7
#define TFT_MISO      GPIO6


#define TFT_CTL_PORT  GPIOB
// PB2, is BOOT1
// PB3, is SDO
#define TFT_CTL_RST   GPIO4
#define TFT_CTL_DC    GPIO5
#define TFT_CTL_LED   GPIO6









static void led_setup(void)
{
  // rcc_periph_clock_enable( RCC_GPIOE );
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}



static inline void delay( uint16_t x )
{
  msleep(x);
}

static uint8_t pgm_read_byte(const uint8_t *addr) {
  return *addr;
}



/*
  read/write != command/data

  p33. p35.

  We use 4 wire. eg. we use D/CX
    If the D/CX bit is “low”, the transmission byte is interpreted as a command byte.
    If the D/CX bit is “high”, the transmission byte is stored as the
                display data RAM (Memory write command), or command register as parameter.

  The 4-line serial mode consists of the
    Data/Command selection input (D/CX),
    chip enable input (CSX),
    the serial clock input (SCL)
    and serial data Input/Output (SDA or SDI/SDO) for data transmission.

  Any instruction can be sent in any order to ILI9341 and the MSB is transmitted first.

  The serial interface is initialized when CSX is high status. In this state,
  SCL clock pulse and SDA data are no effect. A falling edge on CSX enables the
  serial interface and indicates the start of data transmission.

  Host processor drives the CSX pin to low and starts by setting the D/CX bit on
  SDA. The bit is read by ILI9341 on the first rising edge of SCL signal. On the
  next falling edge of SCL, the MSB data bit (D7) is set on SDA by the host. On
  the next falling edge of SCL, the next bit (D6) is set on SDA. If the optional
  D/CX signal is used, a byte is eight read cycle width.
  ---------

  read is defined on p38.
  lookks like supports an 8 bit read.

*/


static void send8( uint8_t x )
{
  spi_send( TFT_SPI, x );
}



static void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes)
{
  gpio_clear(TFT_SPI_PORT, TFT_CS);     // CS active low
  delay(1);

  gpio_clear( TFT_CTL_PORT, TFT_CTL_DC);    // low == command
  delay(1);

  send8(command);
  delay(1);

  gpio_set( TFT_CTL_PORT, TFT_CTL_DC);    // high == data
  delay(1);

  for(unsigned i = 0; i < numDataBytes; ++i) {
    send8(dataBytes[ i ]);
  }
}


static void sendCommand0(uint8_t command)
{

  // gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  gpio_clear( TFT_CTL_PORT, TFT_CTL_DC);    // low == command
  delay(1);


  send8(command);
  delay(1);
}

static void sendData0(uint8_t data)
{
  // advantage is that it can be done in a loop. without allocating stack for buffer.
  // gpio_set(LCD_PORT, LCD_RS);     // high - to assert data
  gpio_set( TFT_CTL_PORT, TFT_CTL_DC);    // high == data
  delay(1);


  send8(data);
  delay(1);
}



///////////////////////////////

#if 0
static void send16( uint16_t x )
{
  send8(x >> 8);   // Check
  send8(x & 0xFF);

}

static void sendCommand16(uint8_t command, uint16_t data)
{

  uint8_t ia[2] = { data >> 8, data & 0xff  } ;

  sendCommand(command, ia, 2);

}



#define LCD_WIDTH 130
#define LCD_HEIGHT 130


#endif



static void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
#if 0
    // column address set
    ILI9341_WriteCommand(0x2A); // CASET
    {
        uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // row address set
    ILI9341_WriteCommand(0x2B); // RASET
    {
        uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // write to RAM
    ILI9341_WriteCommand(0x2C); // RAMWR
#endif
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

    //

    // write to RAM
    // ILI9341_WriteCommand(0x2C); // RAMWR
    // sendCommand(ILI9341_RAMWR, 0 , 0 ); // 2C
}



static void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    // if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT))
    //  return;


    ILI9341_SetAddressWindow(x, y, x+20, y+20);
    // uint8_t data[] = { color >> 8, color & 0xFF };


    // send command
    sendCommand0(ILI9341_RAMWR ); // 2C

    int i;
    for( i = 0; i < 20 * 20; ++i) {
      sendData0( color >> 8 );
      sendData0( color & 0xFF );
    }
}


#if 0
static uint16_t min(uint16_t a, uint16_t b) {
  return (a > b) ? b : a;
}

static uint16_t max(uint16_t a, uint16_t b) {
  return (a > b) ? a : b;
}

static uint16_t sat(uint16_t val, uint16_t vmin, uint16_t vmax) {
  return min(vmax, max(vmin, val));
}

static void lcd_set_cursor(uint16_t x, uint16_t y) {
  sendCommand16(0x20, x);
  sendCommand16(0x21, y);
}


static void lcd_set_window(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom) {
  sendCommand16(0x50, sat(left, 0, LCD_WIDTH-1));
  sendCommand16(0x51, sat(right-1, 0, LCD_WIDTH-1));
  sendCommand16(0x52, sat(top, 0, LCD_HEIGHT-1));
  sendCommand16(0x53, sat(bottom-1, 0, LCD_HEIGHT-1));
}

static void lcd_fill(uint32_t color) {
  // uint16_t data = lcd_pixel_from_rgb32(color);
  // uint16_t data = 0xf7f7;
  // uint16_t data = 0x2200;

  lcd_set_window(0, 0, 320 , 200);
  lcd_set_cursor(0, 0);


  // gpio_clear(LCD_PORT, LCD_RS);   // low - to assert register, D/CX  p24
  gpio_clear( TFT_CTL_PORT, TFT_CTL_DC);    // low == command
  delay(1);
  send8(0x22);
  delay(1);

  // gpio_set(LCD_PORT, LCD_RS);   // high data
  gpio_set( TFT_CTL_PORT, TFT_CTL_DC);    // high == data
  delay(1);

  for(uint32_t i = 0; i < LCD_WIDTH*LCD_HEIGHT; i++) {
      // _lcd_tx_data(data);
      send16(i + 999 );
  }
}

#endif

/////////////////////






static void tft_setup( void )
{
  // uart_printf("dac setup spi\n\r");

  // TODO change GPIOA to TFT_SPI_PORT
  // albeit, should probabaly also do TFT_PORT_AF
  // spi alternate function 5
  gpio_mode_setup(TFT_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TFT_CLK | TFT_MOSI | TFT_MISO );

  // OK.. THIS MADE SPI WORK AGAIN....
  // need harder edges for signal integrity. or else different speed just helps suppress parasitic components
  // see, https://www.eevblog.com/forum/microcontrollers/libopencm3-stm32l100rc-discovery-and-spi-issues/
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TFT_CLK | TFT_MOSI | TFT_MISO );
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, TFT_CLK | TFT_MOSI | TFT_MISO );

  // WARNING - CAREFULl THIS IS SPECFICIC to GPIOA....
  gpio_set_af(GPIOA, GPIO_AF5, TFT_CLK | TFT_MOSI | TFT_MISO );


  // rcc_periph_clock_enable(RCC_SPI1);
  spi_init_master(TFT_SPI,
    SPI_CR1_BAUDRATE_FPCLK_DIV_4,
    // SPI_CR1_BAUDRATE_FPCLK_DIV_256,
    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
    // SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE , // possible we want clock high instead... no doesn't work
    // SPI_CR1_CPHA_CLK_TRANSITION_2,    // 1 == rising edge, 2 == falling edge.
    SPI_CR1_CPHA_CLK_TRANSITION_1,    // 1 == rising edge, 2 == falling edge.
    SPI_CR1_DFF_8BIT,
    SPI_CR1_MSBFIRST
    // SPI_CR1_LSBFIRST
  );
  spi_enable_ss_output(TFT_SPI);
  spi_enable(TFT_SPI);


  // make spi cs regular gpio
  gpio_mode_setup(TFT_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TFT_CS );


  // set up gpio
  gpio_mode_setup(TFT_CTL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TFT_CTL_RST | TFT_CTL_DC | TFT_CTL_LED);

  // uart_printf("dac setup spi done\n\r");
}


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
  // ILI9341_DISPOFF  , 0x80,                // Display on  OK. this definitely appears to do something
  0x00                                   // End of list
};
// clang-format on






int main(void)
{

  rcc_periph_clock_enable( RCC_GPIOE );

  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable( RCC_GPIOA );
  rcc_periph_clock_enable( RCC_GPIOB );


  clock_setup();
  led_setup();
  tft_setup();


  // turn led/backlight on.
  gpio_set( TFT_CTL_PORT, TFT_CTL_LED);    // high

  // assert chip select, with low
  gpio_clear(TFT_SPI_PORT, TFT_CS);       // cs is spi port. this is hard.


  //
  msleep(1000);

  // hardware reset - review
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);    // high
  msleep(150);
  gpio_clear(TFT_CTL_PORT, TFT_CTL_RST);   // low
  msleep(150);
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);   // high
  msleep(150);



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

  
#if 1
  // lcd_fill(0x2200);

  ILI9341_DrawPixel(50, 50, 0xf777 );
#endif

 	while (1) {
    gpio_toggle(GPIOE, GPIO0);
    msleep(500);
	}



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...
  return 0;
}


