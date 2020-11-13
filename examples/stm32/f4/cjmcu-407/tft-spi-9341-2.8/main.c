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
    SPI_CR1_CPHA_CLK_TRANSITION_2,    // 1 == rising edge, 2 == falling edge.
    SPI_CR1_DFF_8BIT,
    SPI_CR1_MSBFIRST
    // SPI_CR1_LSBFIRST
  );
  spi_enable_ss_output(TFT_SPI);
  spi_enable(TFT_SPI);


  // make spi cs regular gpio
  gpio_mode_setup(TFT_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TFT_CS );

  // set up gpio
  gpio_mode_setup(TFT_CTL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PUPD_NONE | TFT_CTL_DC | TFT_CTL_LED);

  // uart_printf("dac setup spi done\n\r");
}






int main(void)
{

  rcc_periph_clock_enable( RCC_GPIOE );

  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable( RCC_GPIOA );
  rcc_periph_clock_enable( RCC_GPIOB );


  clock_setup();
  led_setup();
  tft_setup();



  gpio_set(  TFT_CTL_PORT, TFT_CTL_LED);    // high

  // assert chip select, with low
  gpio_clear(TFT_SPI_PORT, TFT_CS);       // cs is spi port. this is hard.


  // hardware reset - review
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);    // high
  msleep(150);
  gpio_clear(TFT_CTL_PORT, TFT_CTL_RST);   // low
  msleep(150);
  gpio_set(  TFT_CTL_PORT, TFT_CTL_RST);   // high
  msleep(150);



 	while (1) {
    gpio_toggle(GPIOE, GPIO0);
    msleep(500);
	}



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...
  return 0;
}


