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
#define TFT_SPI       SPI1
#define TFT_PORT_SPI  GPIOA

#define TFT_CS        GPIO4
#define TFT_CLK       GPIO5
#define TFT_MOSI      GPIO7
#define TFT_MISO      GPIO6


#define TFT_PORT      GPIOB
// PB2, is BOOT1
// PB3, is SDO
#define TFT_RST       GPIO4
#define TFT_DC        GPIO5
#define TFT_LED       GPIO6








// LCD

static void led_setup(void)
{
  // rename LED_PORT...
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




int main(void)
{
  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOD );

/*

  // gpio_mode_setup(TFT_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0xff ); // JA first 8 bits.
  gpio_mode_setup(TFT_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7  ); // JA first 8 bits.
  gpio_mode_setup(TFT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TFT_RST | TFT_CS | TFT_RS | TFT_WR | TFT_RD);


  gpio_set(TFT_PORT, TFT_RD);   // turn read off. operates both the transceiver and the lcd which reads on rising edge.
                                // when set to read - then if gpio is output - it will sink all the output voltage. very bad.
                                // screen flashing resulted from drop in power supply
*/

  // assert chip select, with low
  gpio_clear(TFT_PORT, TFT_CS);


  // hardware reset - review
  gpio_set(  TFT_PORT, TFT_RST);    // high
  msleep(150);
  gpio_clear(TFT_PORT, TFT_RST);   // low
  msleep(150);
  gpio_set(  TFT_PORT, TFT_RST);   // high
  msleep(150);



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...
  return 0;
}


