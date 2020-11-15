


#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"
#include "Adafruit_ILI9341.h"



#include "lcd_spi.h"
#include "context.h"
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



  Context   ctx;

  // low level
  initialize(&ctx);
  ILI9341_setRotation(&ctx, 3); // 0 == trhs, 1 == brhs, 2 == blhs,  3 == tlhs


  // gfx
  fillScreen(&ctx, ILI9341_WHITE );
  fillRect(&ctx, 100, 50, 200, 70, ILI9341_BLUE );
  fillRect(&ctx, 20, 20, 20, 20, ILI9341_RED );

  writeFastHLine(&ctx, 50, 40, 100, ILI9341_BLUE); 


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


