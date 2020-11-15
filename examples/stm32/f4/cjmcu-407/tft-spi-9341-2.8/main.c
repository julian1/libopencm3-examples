/*
  spi 9341  2.8 with touch

  -- OK. needs 5V - for backlight to be brighterjj
  -- i think.
    think the board LED pin controls mosfets that then turn the backlight on/off.

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
    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPILCD.cpp

  - gfx library - higher level
      https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.cpp

    https://controllerstech.com/interface-tft-display-with-stm32/

  -------
  agg antigrain.
    to use - would need buffer in local memory - because must know background in order to blend pixel.
    so even if use spi - to write the extents. still need backgroun buffer.
    need to modify font to handle - fix paths.

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



#include "lcd_spi.h"
#include "gfx.h"





// tft is one way to create tft


static void led_setup(void)
{
  // rcc_periph_clock_enable( RCC_GPIOE );
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}




/////////////////////








int main(void)
{

  rcc_periph_clock_enable(RCC_GPIOE);

  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  clock_setup();
  led_setup();
  lcd_spi_setup();


  turn_on_backlight( );

  // do once
  // assert_cs();

  //msleep(1000);

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
  ILI9341_DrawRectangle(50, 20, 20, 20, ILI9341_WHITE );

  // blink led
 	while (1) {
    gpio_toggle(GPIOE, GPIO0);

    // gpio_toggle( LCD_CTL_PORT, LCD_CTL_LED);    // toggle backlight
    msleep(500);
	}



  // not sure that the correct commands and data are being sent...
  // display off, or changing the brightness should have done something.

  // OK. nothing looks like its working...
  return 0;
}


