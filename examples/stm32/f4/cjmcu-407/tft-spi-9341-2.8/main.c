/*
  spi 9341  2.8 with touch

  -- OK. needs 5V - for backlight to be brighterjj
    not sure if the LED pin controls mosfets that turn the backlight on/off.


  just connecting led/BL to +3.3V Vcc and backlight turns on.
    probably a digital ctrl.  but maybe drawing current.
    so control with gpio later.

  looks like a couple of fets on the back of pcb. but maybe 5V power supplies.
  no tranceivers


  ------------

  - datasheet ILI9341
    https://www.displayfuture.com/Display/datasheet/controller/ILI9341.pdf


  libopencm3 example uses ILI9341  in spi mode.
    https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f4/stm32f429i-discovery/lcd-dma/lcd-spi.c

  simple,
    https://github.com/adafruit/Adafruit_ILI9341

  primitives
    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.cpp


  - gfx library - higher level 
      https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.cpp

    https://controllerstech.com/interface-tft-display-with-stm32/

  8 bit parallel, has init sequence.
    https://github.com/sammyizimmy/ili9341/blob/master/ili9341.c

  optimised. 70 forks. uses spi.
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
  // millisecond delay
  msleep(x);
}

static inline void nop_sleep( uint32_t n)
{
  uint32_t i;
  for(i = 0; i < n; ++i)  {
    __asm__("nop");
  }
}







static void tft_spi_setup( void )
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


static inline void spi_wait_until_not_busy(uint32_t spi)
{
  /*
    see,
    http://libopencm3.org/docs/latest/stm32f4/html/spi__common__all_8c_source.html#l00194
  */
  /* Wait until not busy */
  while (SPI_SR(spi) & SPI_SR_BSY);
}

static inline void spi_wait_for_transfer_finish(uint32_t spi)
{
   /* Wait for transfer finished. */
   while (!(SPI_SR(spi) & SPI_SR_TXE));


}


static inline void wait_for_transfer_finish(void)
{
/*
  see example, that also uses a loop.
  https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f4/stm32f429i-discovery/lcd-dma/lcd-spi.c
*/
  spi_wait_for_transfer_finish(TFT_SPI);
  // nop_sleep(15);   // 9 doesn't work. 10 does... weird margin

  spi_wait_until_not_busy(TFT_SPI);

}


static inline void send8( uint8_t x )
{
  spi_send( TFT_SPI, x );
}



static inline void assert_cs(void)
{
  // assert chip select, with low
  gpio_clear(TFT_SPI_PORT, TFT_CS);       // cs is spi port. this is hard.
}



static inline void set_command(void )
{
  wait_for_transfer_finish();
  gpio_clear( TFT_CTL_PORT, TFT_CTL_DC);    // low == command
}



static inline void set_data(void )
{
  wait_for_transfer_finish();
  gpio_set( TFT_CTL_PORT, TFT_CTL_DC);    // high == data
}



static void sendCommand(uint8_t command, const uint8_t *dataBytes, uint8_t numDataBytes)
{
  set_command();
  send8(command);

  set_data();
  for(unsigned i = 0; i < numDataBytes; ++i) {
    send8(dataBytes[ i ]);
  }


/*
  EXTREME
    // OK. think issue may be that send8 returns before it has finished sending bytes.
    // so we have to find another call to block until finished.
    // or check register in a loop ourselves.
    // not sure.
    // eg. it returns early. and only blocks if we try to write another byte while still sending spi data.
    // so if we try to fiddle with the command/data register  in the middle of sending it screws up.

*/
}


static void sendCommand0(uint8_t command)
{
  set_command();
  send8(command);
}


#if 0
static void sendData0(uint8_t data)
{
  set_data();
  // delay(1);  // seems not required


  send8(data);
  //delay(1); no required.
}
#endif



/////////////////////





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




static void initialize( void)
{
  // hardware reset - review
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);    // high
  delay(150);
  gpio_clear(TFT_CTL_PORT, TFT_CTL_RST);   // low
  delay(150);
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);   // high
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


static void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
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




static void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t x_off, uint16_t y_off, uint16_t color)
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





#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

// OK. we are going to need a struct to handle this...

uint8_t rotation  ;
uint16_t _width;
uint16_t _height;

static void ILI9341_setRotation(uint8_t m) {

  rotation = m % 4; // can't be higher than 3

  switch (rotation) {
  case 0:
    m = (MADCTL_MX | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 1:
    m = (MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  case 2:
    m = (MADCTL_MY | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 3:
    m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  }

  sendCommand(ILI9341_MADCTL, &m, 1);
}





int main(void)
{

  rcc_periph_clock_enable(RCC_GPIOE);

  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  clock_setup();
  led_setup();
  tft_spi_setup();


  // turn led/backlight on.
  gpio_set( TFT_CTL_PORT, TFT_CTL_LED);    // high

  assert_cs();

  msleep(1000);

  initialize();

  ILI9341_setRotation(3); // 0 == trhs, 1 == brhs, 2 == blhs,  3 == tlhs


  /*
  {
      uint8_t data[] = { 0xff  };
      // sendCommand(ILI_WR_BRIGHTNESS, data, 1 );
      sendCommand(0x51 , data, 1 );
  }
  */



  ILI9341_DrawRectangle(100, 50, 200, 70, ILI9341_BLUE );

  ILI9341_DrawRectangle(20, 20, 20, 20, ILI9341_RED );

  // blink led
 	while (1) {
    gpio_toggle(GPIOE, GPIO0);

    // gpio_toggle( TFT_CTL_PORT, TFT_CTL_LED);    // toggle backlight
    msleep(500);
	}



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...
  return 0;
}


