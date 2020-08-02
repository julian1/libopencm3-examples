/*

spi2
  pb12- pb15  pb9,pb10,   pe14,pd14,pi2,pi3

  pb12 - nss  - cs   blue.  slave select.
  pb13 - sck.  white
  pb14 - miso - grey
  pb15 - mosi - purple

  pe11 - wr - orange 
  pe12 - rs - yellow
  pe13 - rst - green

  we need to map these by name. #define WR PE
  actually not sure. it's messy given the 
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




static void lcd_command(uint8_t cmd, int delay, int n_args, const uint8_t *args)
{
#if 0
	int i;

	gpio_clear(GPIOC, GPIO2);	/* Select the LCD */
	(void) spi_xfer(LCD_SPI, cmd);
	if (n_args) {
		gpio_set(GPIOD, GPIO13);	/* Set the D/CX pin */
		for (i = 0; i < n_args; i++) {
			(void) spi_xfer(LCD_SPI, *(args+i));
		}
	}
	gpio_set(GPIOC, GPIO2);		/* Turn off chip select */
	gpio_clear(GPIOD, GPIO13);	/* always reset D/CX */
	if (delay) {
		msleep(delay);		/* wait, if called for */
	}
#endif

  int i;

	gpio_clear(GPIOB, LCD_NSS);	/* Select the LCD */  // pull low
	(void) spi_xfer(LCD_SPI, cmd);
	if (n_args) {
		gpio_set(GPIOE, LCD_RS);	/* Set the D/CX pin */
		for (i = 0; i < n_args; i++) {
			(void) spi_xfer(LCD_SPI, *(args+i));  // IMPORTANT **** ignores first argument????? *******
		}
	}
	gpio_set(GPIOB, LCD_NSS);		/* Turn off chip select */ // pull high
	gpio_clear(GPIOE, LCD_RS);	/* always reset D/CX */ // pull low
	if (delay) {
		msleep(delay);		/* wait, if called for */
	}

}



// https://github.com/ImpulseAdventure/Waveshare_ILI9486/blob/master/src/Waveshare_ILI9486.cpp

void initializeLcd()
 {
#if 0
  //  Trigger hardware reset.
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delayMicroseconds(20);
  digitalWrite(LCD_RST, HIGH);

  //  TO-DO - how long after a reset until the screen can be used?  Doesn't seem to be
  //  specified in the datasheet.
  //  Experimentally, any less than this and the initial screen clear is incomplete.
  delay(65);
#endif
  
  // appears to do the right thing on the pins. but lcd doesn't change.

  gpio_set(GPIOE, LCD_RST);   // high
  msleep(5);  // milli
  gpio_clear(GPIOE, LCD_RST); // low
  msleep(5);  // milli
  gpio_set(GPIOE, LCD_RST);   // high
  msleep(65);  // milli
}



int main(void)
{

  clock_setup();
  led_setup();

  rcc_periph_clock_enable( RCC_GPIOE );
  rcc_periph_clock_enable( RCC_GPIOB );


  // general registers
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_WR | LCD_RS | LCD_RST ); 

  // 
  // OK. in the example lcd-spi.c it looks like only mosi and clk are set AF. not cs. or miso.
  // spi2 is AF5
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, LCD_SCK | LCD_MOSI );
  gpio_set_af(GPIOB, GPIO_AF5, LCD_SCK | LCD_MOSI );

  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_NSS); 


  rcc_periph_clock_enable(RCC_SPI2);
  spi_init_master(LCD_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
    SPI_CR1_CPHA_CLK_TRANSITION_1,
    SPI_CR1_DFF_8BIT,
    SPI_CR1_MSBFIRST);
  spi_enable_ss_output(LCD_SPI);
  spi_enable(LCD_SPI);


  gpio_clear(GPIOE, LCD_WR); // pull low, select WR for board transceivers



  initializeLcd();

	while (1) {
    gpio_toggle(GPIOE, GPIO0);  // toggle led
    msleep(300);
	}

  return 0;
}


