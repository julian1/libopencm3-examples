/*
  8-bit parallel.


  - general 
    https://controllerstech.com/interface-tft-display-with-stm32/

  https://www.displayfuture.com/Display/datasheet/controller/ILI9341.pdf

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

// GPIOB
#define LCD_NSS   GPIO12 // not AF
#define LCD_SCK   GPIO13
#define LCD_MISO  GPIO14 // not used
#define LCD_MOSI  GPIO15

// GPIOE
#define LCD_WR    GPIO11
#define LCD_RS    GPIO12
#define LCD_RST   GPIO13

#define LCD_SPI   SPI2

static void led_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOE); // JA
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // JA
}


// should put control registers on different port - this would allow 16 bit parallel bit bashing. if we wanted 

int main(void)
{
  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOB );
  rcc_periph_clock_enable( RCC_GPIOE );


  // gpio_set( GPIOB, 0b1000000000100000 << 16 );   // should set pa5 and pa15
  gpio_set( GPIOB, 0b1000000000100000  );   // should set pa5 and pa15, really need to test.

	while (1) {
    gpio_toggle(GPIOE, GPIO0);  // toggle led
    msleep(300);
	}

  return 0;
}


